
import time
import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 0
MOVE_DURATION = 3.0        # seconds to complete motion (increase = slower)
FREQUENCY = 500
DT = 1 / FREQUENCY
GEAR_RATIO = 9.0
STEP_SIZE = 1/2 * np.pi    # π/2 step


def position_control():
    position_logger = Logger(
        log_path="./logs",
        file_name="position_control",
    )

    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.POSITION)
        actpack.set_position_gains()

        actpack.update()
        start_position = actpack.output_position
        command_position = start_position

        move_started = False
        move_start_time = 0.0

        position_logger.track_function(lambda: actpack.output_position, "Output Position")
        position_logger.track_function(lambda: command_position, "Command Position")
        position_logger.track_function(lambda: time.time(), "Time")

        for t in clock:

            # Start movement after TIME_TO_STEP
            if (t > TIME_TO_STEP) and not move_started:
                move_started = True
                move_start_time = t
                target_position = start_position + STEP_SIZE

            if move_started:
                elapsed = t - move_start_time
                progress = min(elapsed / MOVE_DURATION, 1.0)

                # Cosine smoothing (ease in / ease out)
                smooth_profile = 0.5 * (1 - np.cos(np.pi * progress))

                command_position = (
                    start_position
                    + STEP_SIZE * smooth_profile
                )

                actpack.set_output_position(value=command_position)

            actpack.update()

            position_logger.info(
                f"Time: {t}; "
                f"Command Position: {command_position}; "
                f"Output Position: {actpack.output_position}"
            )

            position_logger.update()


if __name__ == "__main__":
    position_control()

