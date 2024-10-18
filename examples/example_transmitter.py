import os
import sys
import time
import RPi.GPIO as GPIO
from loguru import logger

#from rpi_sx1280 import const
#from rpi_sx1280.sx1280 import SX128XLT
import const
from sx1280 import SX128XLT

#zero@zero.local
lora: SX128XLT = None

txPower = 10

freq = 2445000000
offset = 0
bandwidth = const.LORA_BW_0400
spreading_factor = const.LORA_SF7
code_rate = const.LORA_CR_4_5


buff = "Hello World 1234567890"


def init():
    global lora

    # configure RPi GPIO
    GPIO.setmode(GPIO.BCM)

    # initialise the SX1280 module
    lora = SX128XLT(0, 0, 8, 22, pin_nreset = 23, pin_rxen = 25,pin_txen = 24, pin_dio1 = 27)

    # setup the LoRa configuration
    lora.setupLoRa(freq, offset, spreading_factor, bandwidth, code_rate)
    lora.setBufferBaseAddress(0, 1)

    # debug configuration values
    lora.printModemSettings()
    lora.printOperatingSettings()
    lora.printRegisters(0x900, 0x9FF)

    logger.info("~~~ LoRa SX1280 Transmitter is Ready ~~~")


def loop():
    global lora

    startMs = time.time()

    txPacketL = lora.transmit(buff, 10, txPower, const.NO_WAIT) #const.WAIT_TX)

    if txPacketL > 0:
        endMs = time.time()

        logger.info(f"TX: {buff} ~ in {endMs - startMs}ms")
    else:
        logger.error("Transmit failed")

    time.sleep(5)


if __name__ == "__main__":
    try:
        init()
        while True:
            loop()
    except KeyboardInterrupt:
        logger.warning("Interrupted")
        GPIO.cleanup()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
    except:
        GPIO.cleanup()
        logger.error("Unexpected error:", sys.exc_info()[0])
