
"""
Dephy Load Cell Calibration and Verification Script

This script interfaces with a Dephy load cell amplifier to perform calibration,
zeroing, and verification measurements both without and with an applied weight. 

Key features:

1. Load Cell Initialization: Connects to the Dephy load cell amplifier using a
   predefined calibration matrix and hardware settings (gain, excitation voltage, bus, address).

2. Calibration (Zeroing): Prompts the user to remove all weight and performs
   zeroing to eliminate any initial bias in the sensor readings.

3. Data Acquisition Without Weight: Collects multiple samples of the vertical
   force (Fz) with no load applied. Applies a simple low-pass filter to reduce
   noise and stores both raw and filtered readings.

4. Data Acquisition With Weight: Prompts the user to apply a known weight,
   then records Fz over multiple samples, again applying the low-pass filter and
   storing raw and filtered data.

5. Data Visualization: Plots Fz over time, comparing raw and filtered signals
   for both no-weight and with-weight conditions. The plot is saved automatically
   as `loadcell_Fz_plot.png`.

6. Filtering: A simple low-pass filter (α = 0.1) is applied to smooth the
   measurements.

This script is useful for testing, verifying, and calibrating the load cell
before integrating it into a larger robotic or experimental setup.
"""

import time
import numpy as np
import matplotlib.pyplot as plt

from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier

# Calibration matrix (from OSL github)
LOADCELL_CALIBRATION_MATRIX = np.array([
    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
    (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
    (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
    (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
    (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
    (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
])

def main():

    loadcell = DephyLoadcellAmplifier(
        calibration_matrix=LOADCELL_CALIBRATION_MATRIX,
        tag="loadcell",
        amp_gain=125,
        exc=5,
        bus=1,
        i2c_address=102,
    )

    # Lists to store data for plotting
    Fz_raw_no_weight = []
    Fz_filtered_no_weight = []
    Fz_raw_weight = []
    Fz_filtered_weight = []

    with loadcell:
        print("Initializing system")
        input("\nRemove all weight and press ENTER to calibrate...")

        # Calibration (zeroing)
        print("Calibrating...")
        loadcell.calibrate(reset=True)
        print("Calibration completed")

        # Verify without weight
        input("\nVerifying (without weight)...")
        fz_filtered = 0.0
        for _ in range(20):
            loadcell.update()
            fz_new = loadcell.fz

            # simple low-pass filter
            fz_filtered = 0.9 * fz_filtered + 0.1 * fz_new

            Fz_raw_no_weight.append(fz_new)
            Fz_filtered_no_weight.append(fz_filtered)

            print(f"Fz raw: {fz_new:.2f} N, Fz filtered: {fz_filtered:.2f} N")
            time.sleep(0.1)

        # Verify with weight
        input("\nNow apply weight and press ENTER...")
        fz_filtered = 50.0
        for _ in range(50):
            loadcell.update()
            fz_new = loadcell.fz

            # simple low-pass filter
            fz_filtered = 0.9 * fz_filtered + 0.1 * fz_new

            Fz_raw_weight.append(fz_new)
            Fz_filtered_weight.append(fz_filtered)

            print(f"Fz raw: {fz_new:.2f} N, Fz filtered: {fz_filtered:.2f} N")
            time.sleep(0.1)

    # --- Plotting ---
    time_no_weight = list(range(len(Fz_raw_no_weight)))
    time_weight = list(range(len(Fz_raw_no_weight), len(Fz_raw_no_weight) + len(Fz_raw_weight)))

    plt.figure(figsize=(12,6))
    # Without weight
    plt.plot(time_no_weight, Fz_raw_no_weight, 'r-o', label='Fz raw (no weight)')
    plt.plot(time_no_weight, Fz_filtered_no_weight, 'b-o', label='Fz filtered (no weight)')
    # With weight
    plt.plot(time_weight, Fz_raw_weight, 'r--', label='Fz raw (with weight)')
    plt.plot(time_weight, Fz_filtered_weight, 'b--', label='Fz filtered (with weight)')

    # Labels and legend
    plt.xlabel('Sample')
    plt.ylabel('Fz (N)')
    plt.title('Load Cell Fz: Raw vs Filtered')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Save plot automatically
    plt.savefig("loadcell_Fz_plot.png")

if __name__ == "__main__":
    main()
