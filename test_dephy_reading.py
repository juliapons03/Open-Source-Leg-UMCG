"""
Knee Actuator Periodic Trajectory Test

This script controls a Dephy knee actuator to follow a periodic cosine trajectory
between 10° and 90° at a fixed frequency. Key features:

1. Fixed Start Position: Before starting the trajectory, the motor moves to a
   predefined starting angle (10°) to ensure repeatable experiments.

2. Periodic Trajectory Execution: The actuator follows a cosine trajectory with
   a defined period (10 seconds), while recording position, velocity, and torque.

3. Cycle Detection: The script detects when the actuator crosses a specified
   angle threshold (50°) to count cycles.

4. Half-Cycle Extension: After completing the requested number of cycles,
   the trajectory continues for approximately half of the next cycle to capture
   smooth motion at the end.

5. Data Logging: The script logs time, angle, velocity, and torque at each
   loop iteration.

6. Plotting: Generates a 3-panel plot (angle, velocity, torque) showing
   only the data from the first detected cycle onward, with cycle crossings
   marked as vertical dashed lines.

This setup ensures consistent and repeatable tests of the actuator's
dynamic behavior under controlled position commands.
"""
import numpy as np
import matplotlib.pyplot as plt
import time
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.utilities import SoftRealtimeLoop, units

FREQUENCY = 200
NUM_CYCLES = 5
ANGLE_THRESHOLD = 50*np.pi/180  # rad, detect cycle when angle crosses threshold
START_ANGLE_DEG = 10

# Initialize knee actuator
knee = DephyActuator(
    tag="knee",
    firmware_version="7.2.0",
    port="/dev/ttyACM0",
    gear_ratio=9 * 83 / 18,
    frequency=FREQUENCY,
)

clock = SoftRealtimeLoop(dt=1 / FREQUENCY)

def make_periodic_trajectory(period, minimum, maximum):
    amplitude = (maximum - minimum) / 2
    mean = amplitude + minimum
    return lambda t: amplitude * np.cos(t * 2 * np.pi / period) + mean

knee_traj = make_periodic_trajectory(10, 10, 90)

# Data storage
time_data = []
angle_data = []
vel_data = []
torque_data = []
cycle_times = []

cycle_count = 0
prev_angle = None

with knee:
    input("Press Enter to continue")

    knee.set_control_mode(CONTROL_MODES.POSITION)
    knee.set_position_gains(kp=5)

    # Move to a fixed starting position
    start_position_rad = units.convert_to_default(START_ANGLE_DEG, units.Position.deg)
    knee.set_output_position(start_position_rad)

    time.sleep(3) # small delay to avoid busy waiting

    print(f"Motor positioned at {START_ANGLE_DEG} deg, starting trajectory...")

    start_time = time.time()
    for t in clock:
        knee.update()

        # Compute setpoint
        knee_setpoint = units.convert_to_default(knee_traj(t), units.Position.deg)
        knee.set_output_position(knee_setpoint)

        # Record actuator data
        angle = knee.output_position
        vel = knee.output_velocity
        torque = knee.output_torque
        now = time.time() - start_time

        time_data.append(now)
        angle_data.append(angle)
        vel_data.append(vel)
        torque_data.append(torque)

        # Detect cycle crossings
        if prev_angle is not None and prev_angle < ANGLE_THRESHOLD <= angle:
            cycle_count += 1
            cycle_times.append(now)
            print(f"Cycle {cycle_count} detected at t = {now:.2f}s")
        prev_angle = angle

        if cycle_count >= NUM_CYCLES:
            time.sleep(1)
            break

        print(f"Knee Desired {knee_setpoint:+.2f} rad, Actual {angle:+.2f} rad", end="\r")

# -----------------------------
# Truncate the data
# -----------------------------

# Find the index of the first cycle
if cycle_times:  # make sure at least one cycle was detected
    first_cycle_time = cycle_times[0]

    # Find the index in time_data closest to the first cycle
    start_idx = next(i for i, t in enumerate(time_data) if t >= first_cycle_time)

    # Truncate all data arrays
    time_data = [t - time_data[start_idx] for t in time_data[start_idx:]]  # reset time to zero at first cycle
    angle_data = angle_data[start_idx:]
    vel_data = vel_data[start_idx:]
    torque_data = torque_data[start_idx:]
    cycle_times = [ct - time_data[0] for ct in cycle_times if ct >= first_cycle_time]  # adjust cycle_times

# -----------------------------
# Plot results
# -----------------------------

plt.figure(figsize=(12,8))

plt.subplot(3,1,1)
plt.plot(time_data, [a*180/np.pi for a in angle_data], 'b')
for ct in cycle_times:
    plt.axvline(ct, color='k', linestyle='--', alpha=0.7)
plt.ylabel("Angle (deg)")
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(time_data, vel_data, 'r')
for ct in cycle_times:
    plt.axvline(ct, color='k', linestyle='--', alpha=0.7)
plt.ylabel("Velocity (rad/s)")
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_data, torque_data, 'g')
for ct in cycle_times:
    plt.axvline(ct, color='k', linestyle='--', alpha=0.7)
plt.ylabel("Torque est.")
plt.xlabel("Time (s)")
plt.grid(True)

plt.tight_layout()
plt.savefig("knee_motor_cycles.png")
print("\n Plot saved as knee_motor_cycles.png")
