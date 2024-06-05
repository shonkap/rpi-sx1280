import time
import RPi.GPIO as GPIO

from loguru import logger
from rpi_sx1280.sx1280 import SX1280

GPIO.setmode(GPIO.BCM)

lora = SX1280(0, 0, 17, 4, debug=True)
lora.set_ranging_params(master=True)

while True:
    lora.get_range()

    time.sleep(4)

    status = lora.get_irq_status()
    if status[2] > 0 or status[3] > 0:
        data1 = lora.read_range(raw=False)
        data2 = lora.read_range(raw=True)

        logger.info(f"Range: {data1}m (raw: {data2})")
