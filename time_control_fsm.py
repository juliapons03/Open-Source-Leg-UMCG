"""
Time-based FSM knee controller for OpenSourceLeg.

- Alternates stance/swing every 5 seconds
- Runs for 40 seconds total
- Logs k and b
- Saves plot
"""

import numpy as np
import matplotlib.pyplot as plt
import os

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier
from opensourceleg.utilities import SoftRealtimeLoop


# ---------------- PARAMETERS ---------------- #

GEAR_RATIO = 9 * (83 / 18)
FREQUENCY = 200

TOTAL_TIME = 40.0
SWITCH_INTERVAL = 5.0

LOADCELL_CALIBRATION_MATRIX = np.array([
    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
    (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
    (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
    (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
    (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
    (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
])


# ---------------- FSM ---------------- #

def create_knee_fsm():

    stance = State(
        name="stance",
        knee_theta=5,
        knee_stiffness=500,
        knee_damping=20,
    )

    swing = State(
        name="swing",
        knee_theta=70,
        knee_stiffness=3,
        knee_damping=0.1,
    )

    fsm = StateMachine(
        states=[stance, swing],
        initial_state_name="stance",
    )

    # time tracking (external clock)
    fsm.current_time = 0.0
    fsm.last_switch_time = 0.0
    fsm.switch_interval = SWITCH_INTERVAL

    # ONE clean transition rule
    def time_to_switch(osl):
        return (fsm.current_time - fsm.last_switch_time) >= fsm.switch_interval

    fsm.add_transition(stance, swing, "to_swing", time_to_switch)
    fsm.add_transition(swing, stance, "to_stance", time_to_switch)

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

    os.makedirs("./logs", exist_ok=True)

    fsm_logger = Logger(
        log_path="./logs",
        file_name="knee_fsm.log",
    )

    osl = OpenSourceLeg(
        tag="osl",
        actuators=actuators,
        sensors=sensors,
    )

    fsm = create_knee_fsm()

    # ---------------- STARTUP ---------------- #

    with osl, fsm:

        print("Initializing system...")
        osl.update()
        osl.home()

        print("Calibrating loadcell...")
        osl.loadcell.reset()
        osl.loadcell.calibrate()

        input("Press Enter to start control...")

        osl.knee.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
        osl.knee.set_impedance_cc_pidf_gains()
        osl.knee.set_output_impedance()

        print("Running...")

        # ---------------- DATA ---------------- #

        time_log = []
        k_log = []
        b_log = []

        prev_state = fsm.current_state.name

        # ---------------- LOOP ---------------- #

        for t in clock:

            if t > TOTAL_TIME:
                break

            osl.update()

            # update FSM time
            fsm.current_time = t
            fsm.update(osl=osl)

            # detect state change 
            if fsm.current_state.name != prev_state:
                print("\a")
                print(f"State changed → {fsm.current_state.name}")
                fsm.last_switch_time = t
                prev_state = fsm.current_state.name

            # impedance
            k = fsm.current_state.knee_stiffness
            b = fsm.current_state.knee_damping

            osl.knee.set_output_impedance(k=k, b=b)

            # logging
            time_log.append(t)
            k_log.append(k)
            b_log.append(b)

            print(f"T:{t:.2f} | {fsm.current_state.name} | k:{k} | b:{b}")

            fsm_logger.info(
                f"T:{t:.3f}, State:{fsm.current_state.name}, Fz:{osl.loadcell.fz:.2f}"
            )

    # ---------------- PLOT ---------------- #

    print("Saving plot...")

    plt.figure()

    plt.subplot(2, 1, 1)
    plt.step(time_log, k_log, where="post")
    plt.ylabel("Stiffness (k)")
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.step(time_log, b_log, where="post")
    plt.ylabel("Damping (b)")
    plt.xlabel("Time (s)")
    plt.grid()

    plt.tight_layout()

    path = "./k_b_plot.png"
    plt.savefig(path, dpi=300)

    print(f"Saved plot → {path}")


    plt.show()
