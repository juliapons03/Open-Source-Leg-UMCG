"""
This script is the same as fsm_test_1.py, but extends it by adding data logging 
and post-processing plots of the impedance parameters over time.
"""

import numpy as np 
import matplotlib.pyplot as plt

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier
from opensourceleg.utilities import SoftRealtimeLoop


# ---------------- PARAMETERS --------------- #

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

BODY_WEIGHT = 10 * 9.8

LOAD_STANCE = 0.25 * BODY_WEIGHT
LOAD_SWING = 0.15 * BODY_WEIGHT


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
        knee_stiffness=30,
        knee_damping=0.1,
    )

    def stance_to_swing(osl):
        return osl.loadcell.fz > -LOAD_SWING

    def swing_to_stance(osl):
        return osl.loadcell.fz < -LOAD_STANCE

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

    # ---------------- DATA LOGGING BUFFERS ---------------- #
    time_log = []
    k_log = []
    b_log = []
    state_log = []

    # ---------------- SYSTEM STARTUP ---------------- #

    with osl, fsm:
        print("Initializing system...")

        osl.update()

        print("Homing system...")
        osl.home()

        print("Calibrating loadcell...")
        osl.loadcell.reset()
        osl.loadcell.calibrate()

        print("System ready.")
        input("Press Enter to start knee control...")

        osl.knee.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        osl.knee.set_impedance_cc_pidf_gains()
        osl.knee.set_output_impedance()

        print("Control loop started.")

        # ---------------- REAL-TIME LOOP ---------------- #

        for t in clock:
            osl.update()
            fsm.update(osl=osl)

            k = fsm.current_state.knee_stiffness
            b = fsm.current_state.knee_damping

            osl.knee.set_output_impedance(k=k, b=b)

            # file logging
            fsm_logger.info(
                f"T:{t:.3f}, "
                f"State:{fsm.current_state.name}, "
                f"Fz:{osl.loadcell.fz:.2f}, "
                f"Knee:{np.rad2deg(osl.knee.output_position):.2f}"
            )

            # plot logging
            time_log.append(t)
            k_log.append(k)
            b_log.append(b)
            state_log.append(fsm.current_state.name)


# ---------------- PLOTTING ---------------- #

time_log = np.array(time_log)
k_log = np.array(k_log)
b_log = np.array(b_log)
state_log = np.array(state_log)

def plot_by_state(y, title, filename):
    plt.figure()

    for i in range(1, len(time_log)):
        style = "-" 

        plt.plot(
            time_log[i-1:i+1],
            y[i-1:i+1],
            style,
            color="black"
        )

    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(title)
    plt.grid(True)

    plt.savefig(filename, dpi=300, bbox_inches="tight")
    plt.close()


# Save plots
plot_by_state(k_log, "Knee stiffness (k)", "knee_stiffness_plot.png")
plot_by_state(b_log, "Knee damping (b)", "knee_damping_plot.png")
