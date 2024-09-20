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


def init():
    global lora

    # configure RPi GPIO
    GPIO.setmode(GPIO.BCM)

    # initialise the SX1280 module
    lora = SX128XLT(0, 0, 8, 22, pin_nreset = 23, pin_rxen = 25,pin_txen = 24, pin_dio1 = 2)
    logger.info("init done")
    # setup the LoRa configuration
    lora.setupLoRa(freq, offset, spreading_factor, bandwidth, code_rate)
    logger.info("init lora half done")
    lora.setBufferBaseAddress(1, 0)
    logger.info("init lora done")
    # debug configuration values
    lora.printModemSettings()
    lora.printOperatingSettings()
    lora.printRegisters(0x900, 0x9FF)

    logger.info("~~~ LoRa SX1280 Receiver is Ready ~~~")


def loop():
    global lora

    # wait for the packet to arrive with 60s timeout
    rxPacket = lora.receive(60, const.WAIT_RX)

    if len(rxPacket) > 0:
        # read RSSI value and SNR value
        packetRSSI = lora.readPacketRSSI()
        packetSNR = lora.readPacketSNR()

        logger.info(f"Received packet with RSSI {packetRSSI} and SNR {packetSNR}")
        logger.info(f"Received packet: {rxPacket}")


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
