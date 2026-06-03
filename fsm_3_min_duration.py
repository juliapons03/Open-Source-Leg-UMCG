"""
Finite state machine (FSM)-based knee control system for the OpenSourceLeg platform.
State transitions are based on FILTERED load cell feedback only.

Encoder is logged but not used for control.

Added:
- Minimum state duration / transition lockout
- Prevents rapid false switching due to noisy force signals
"""

import numpy as np
import matplotlib.pyplot as plt

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.utilities import SoftRealtimeLoop


# ---------------- PARAMETERS ---------------- #

GEAR_RATIO = 9 * (83 / 18)
FREQUENCY = 200

LOADCELL_CALIBRATION_MATRIX = np.array([
    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
    (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
    (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
    (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
    (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
    (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
])

BODY_WEIGHT = 60 * 9.8

LOAD_STANCE = 0.20 * BODY_WEIGHT
LOAD_SWING = 0.05 * BODY_WEIGHT

# Minimum time before another transition is allowed
# Recommended values: 0.2 - 0.5 s
MIN_STATE_TIME = 0.2


# ---------------- FSM ---------------- #

def create_knee_fsm(osl: OpenSourceLeg) -> StateMachine:

    stance = State(
        name="stance",
        knee_theta=5,
        knee_stiffness=500,
        knee_damping=20,
    )

    swing = State(
        name="swing",
        knee_theta=60,
        knee_stiffness=10,
        knee_damping=0.5,
    )

    # Store last transition time
    osl.last_state_change_time = 0.0

    # ---------------- TRANSITION LOCKOUT ---------------- #

    def transition_allowed(osl):

        current_time = osl.clock_time

        return (
            current_time - osl.last_state_change_time
        ) >= MIN_STATE_TIME

    # ---------------- TRANSITIONS ---------------- #

    def stance_to_swing(osl):

        # Block transition if lockout active
        if not transition_allowed(osl):
            return False

        if osl.filtered_fz > -LOAD_SWING:

            osl.last_state_change_time = osl.clock_time

            return True

        return False

    def swing_to_stance(osl):

        # Block transition if lockout active
        if not transition_allowed(osl):
            return False

        if osl.filtered_fz < -LOAD_STANCE:

            osl.last_state_change_time = osl.clock_time

            return True

        return False

    fsm = StateMachine(
        states=[stance, swing],
        initial_state_name="stance",
    )

    fsm.add_transition(
        source=stance,
        destination=swing,
        event_name="toe_off",
        criteria=stance_to_swing,
    )

    fsm.add_transition(
        source=swing,
        destination=stance,
        event_name="heel_strike",
        criteria=swing_to_stance,
    )

    return fsm


# ---------------- MAIN ---------------- #

if __name__ == "__main__":

    actuators = {
        "knee": DephyActuator(
            tag="knee",
            port="/dev/ttyACM0",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            debug_level=0,
            dephy_log=False,
        ),
    }

    sensors = {
        "loadcell": DephyLoadcellAmplifier(
            calibration_matrix=LOADCELL_CALIBRATION_MATRIX,
            tag="loadcell",
            amp_gain=125,
            exc=5,
            bus=1,
            i2c_address=102,
        ),

        "encoder": AS5048B(
            tag="encoder1",
            bus="/dev/i2c-1",
            A1_adr_pin=True,
            A2_adr_pin=False,
            zero_position=0,
            enable_diagnostics=False,
        ),
    }

    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)

    fsm_logger = Logger(
        log_path="./logs",
        file_name="knee_fsm.log",
    )

    osl = OpenSourceLeg(
        tag="osl",
        actuators=actuators,
        sensors=sensors,
    )

    fsm = create_knee_fsm(osl)

    # ---------------- LOW PASS FILTER ---------------- #

    CUTOFF_HZ = 10
    DT = 1 / FREQUENCY

    alpha = DT / (1 / (2 * np.pi * CUTOFF_HZ) + DT)

    fz_filtered = 0.0

    # ---------------- LOGS ---------------- #

    time_log = []

    k_log = []
    b_log = []
    state_log = []

    theta_log = []
    theta_ref_log = []

    encoder_theta_log = []
    encoder_velocity_log = []

    loadcell_fz_log = []

    loadcell_fz_filt_log = []

    # ---------------- START ---------------- #

    with osl, fsm:

        print("Starting system...")
        osl.update()

        print("Homing...")
        osl.home()

        print("Calibrating loadcell...")
        osl.loadcell.reset()
        osl.loadcell.calibrate()

        input("Press Enter to start control...")

        osl.knee.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        osl.knee.set_impedance_cc_pidf_gains()
        osl.knee.set_output_impedance()

        try:
            for t in clock:

                osl.update()

                # ---------------- FILTER ---------------- #

                raw_fz = osl.loadcell.fz

                fz_filtered = (
                    alpha * raw_fz
                    + (1 - alpha) * fz_filtered
                )

                osl.filtered_fz = fz_filtered

                # ---------------- FSM ---------------- #

                # Store current loop time
                osl.clock_time = t

                fsm.update(osl=osl)

                # ---------------- CONTROL ---------------- #

                k = fsm.current_state.knee_stiffness
                b = fsm.current_state.knee_damping

                theta_ref = np.deg2rad(
                    fsm.current_state.knee_theta
                )

                theta_actual = osl.knee.output_position

                encoder_theta = osl.sensors["encoder"].position
                encoder_velocity = osl.sensors["encoder"].velocity

                osl.knee.set_output_impedance(k=k, b=b)

                osl.knee.set_motor_position(theta_ref)

                # ---------------- LOG ---------------- #

                fsm_logger.info(
                    f"T:{t:.3f}, "
                    f"State:{fsm.current_state.name}, "
                    f"Fz:{fz_filtered:.2f}, "
                    f"ThetaRef:{np.rad2deg(theta_ref):.2f}, "
                    f"ThetaActual:{np.rad2deg(theta_actual):.2f}, "
                    f"EncoderTheta:{np.rad2deg(encoder_theta):.2f}, "
                    f"EncoderVelocity:{np.rad2deg(encoder_velocity):.2f}, "
                    f"K:{k:.2f}, "
                    f"B:{b:.2f}"
                )

                # ---------------- STORE ---------------- #

                time_log.append(t)

                k_log.append(k)
                b_log.append(b)

                state_log.append(
                    fsm.current_state.name
                )

                theta_log.append(
                    np.rad2deg(theta_actual)
                )

                theta_ref_log.append(
                    np.rad2deg(theta_ref)
                )

                encoder_theta_log.append(
                    np.rad2deg(encoder_theta)
                )

                encoder_velocity_log.append(
                    np.rad2deg(encoder_velocity)
                )

                loadcell_fz_log.append(raw_fz)

                loadcell_fz_filt_log.append(
                    fz_filtered
                )

        except KeyboardInterrupt:
            print("\nStopped.")


# ---------------- CONVERT ---------------- #

time_log = np.array(time_log)

k_log = np.array(k_log)
b_log = np.array(b_log)

state_log = np.array(state_log)

theta_log = np.array(theta_log)
theta_ref_log = np.array(theta_ref_log)

encoder_theta_log = np.array(encoder_theta_log)
encoder_velocity_log = np.array(encoder_velocity_log)

loadcell_fz_log = np.array(loadcell_fz_log)
loadcell_fz_filt_log = np.array(loadcell_fz_filt_log)


# ---------------- PLOT ---------------- #

# ---------------- THETA ---------------- #

plt.figure(figsize=(12, 4))

plt.plot(
    time_log,
    theta_log,
    label="Actuator Knee Angle",
    linewidth=2
)

plt.plot(
    time_log,
    theta_ref_log,
    "--",
    label="Reference Knee Angle",
    linewidth=2
)

plt.title("Knee Angle Tracking Over Time (Min state duration: 0.2s)")

plt.xlabel("Time (s)")
plt.ylabel("Theta (deg)")

plt.grid(True)
plt.legend(loc="center left",bbox_to_anchor=(1, 0.5))

plt.savefig(
    "knee_theta_plot.png",
    dpi=300,
    bbox_inches="tight"
)

# ---------------- ENCODER + LOADCELL ---------------- #

fig, axs = plt.subplots(
    3,
    1,
    figsize=(12, 10),
    sharex=True
)

axs[0].plot(
    time_log,
    encoder_theta_log
)

axs[0].set_title("Encoder Angle")
axs[0].set_ylabel("Angle (deg)")
axs[0].grid(True)

axs[1].plot(
    time_log,
    encoder_velocity_log
)

axs[1].set_title("Encoder Velocity")
axs[1].set_ylabel("Velocity (deg/s)")
axs[1].grid(True)

axs[2].plot(
    time_log,
    loadcell_fz_log,
    label="Raw"
)

axs[2].plot(
    time_log,
    loadcell_fz_filt_log,
    label="Filtered"
)

axs[2].set_title("Loadcell Force")
axs[2].set_xlabel("Time (s)")
axs[2].set_ylabel("Fz (N)")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()

plt.savefig(
    "encoder_loadcell_outputs_plot.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()

print("Saved all plots:")

print("- knee_theta_plot.png")
print("- encoder_loadcell_outputs_plot.png")
