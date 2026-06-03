"""
AS5048B Encoder Data Logging Script

Continuously reads position data from an AS5048B magnetic encoder and logs it
at 100 Hz using OpenSourceLeg's Logger.

Features:
- Initializes the encoder with specified I2C address pins and zero position.
- Uses a real-time loop to sample the encoder angle.
- Logs both raw position and angle in degrees.
- Stores logs in ./logs/reading_encoder_data.
- Frequency reduced from 1000 Hz to 100 Hz for visual observation.
"""

from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.utilities import SoftRealtimeLoop
import math

FREQUENCY = 100  #Before it was 1000Hz, but too fast to visully see the output
DT = 1 / FREQUENCY

if __name__ == "__main__":
    encoder_logger = Logger(
        log_path="./logs",
        file_name="reading_encoder_data",
    )
    clock = SoftRealtimeLoop(dt=DT)
    encoder = AS5048B(
        tag="encoder1",
        bus="/dev/i2c-1",
        A1_adr_pin=True,
        A2_adr_pin=False,
        zero_position=0,
        enable_diagnostics=False,
    )
    encoder_logger.track_function(lambda: encoder.position, "Encoder Position")

    with encoder:
        for t in clock:
            encoder.update()
            encoder_logger.info(f"Time: {t}; Encoder Angle: {math.degrees(encoder.position)};")
            encoder_logger.update()