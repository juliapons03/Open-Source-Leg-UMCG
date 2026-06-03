"""
A basic motion script that moves the osl joints through their range of motion.
This script can be helpful when getting started to make sure the OSL is functional.

Kevin Best
Neurobionics Lab
Robotics Department
University of Michigan
October 26, 2023
"""

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.utilities import SoftRealtimeLoop, units

FREQUENCY = 200

knee = DephyActuator(
    tag="knee",
    firmware_version="7.2.0",
    port="/dev/ttyACM0",
    gear_ratio=9 * 83 / 18,
    frequency=FREQUENCY,
)

acutators = [knee]

clock = SoftRealtimeLoop(dt=1 / FREQUENCY)

def make_periodic_trajectory(period, minimum, maximum):
    amplitude = (maximum - minimum) / 2
    mean = amplitude + minimum
    return lambda t: amplitude * np.cos(t * 2 * np.pi / period) + mean

knee_traj = make_periodic_trajectory(10, 10, 90)

with knee:
    
    input("Homing complete: Press enter to continue")

    knee.set_control_mode(CONTROL_MODES.POSITION)
    knee.set_position_gains(kp=5)

    for t in clock:
        knee.update()
        
        knee_setpoint = units.convert_to_default(knee_traj(t), units.Position.deg)
        
        knee.set_output_position(knee_setpoint)
        
        print(
            f"Knee Desired {knee_setpoint:+.2f} rad, Ankle Desired {knee.output_position:+.2f} rad",
            end="\r",
        )

print("\n")