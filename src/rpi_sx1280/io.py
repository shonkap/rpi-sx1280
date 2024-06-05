import spidev
import RPi.GPIO as GPIO
from loguru import logger


class SpiDevice:
    def __init__(self, spi_id, cs_id, debug=False):
        self._debug = debug

        self._device = spidev.SpiDev()
        self._spi_id = spi_id
        self._cs_id = cs_id

    def __enter__(self):
        self._device.open(self._spi_id, self._cs_id)

        self._device.mode = 0b00
        self._device.max_speed_hz = 18_000_000

        if self._debug:
            logger.debug(
                f"SPI device opened on SPI{self._spi_id} with CS{self._cs_id}, mode: {self._device.mode}, speed: {self._device.max_speed_hz/1_000_000}MHz"
            )

        return self._device

    def __exit__(self, exc_type, exc_value, exc_tb):
        self._device.close()

        if self._debug:
            logger.debug(f"SPI device closed on SPI{self._spi_id} with CS{self._cs_id}")

        return self._device


class IOPin:
    def __init__(self, pin_id, direction, init=None):
        self._pin = pin_id
        self._dir = direction

        if direction == GPIO.OUT:
            GPIO.setup(self._pin, self._dir, initial=init)
        else:
            GPIO.setup(self._pin, self._dir)
            GPIO.input(self._pin)

    def read(self):
        if self._dir == GPIO.IN:
            return GPIO.input(self._pin)
        else:
            logger.error("IO direction does not allow read() operation")

    def write(self, state):
        if self._dir == GPIO.OUT:
            if state != GPIO.HIGH and state != GPIO.LOW:
                logger.error("State value not supported")
            else:
                GPIO.output(self._pin, state)
        else:
            logger.error("IO direction does not allow write() operation")

    def low(self):
        if self._dir == GPIO.OUT:
            GPIO.output(self._pin, GPIO.LOW)
        else:
            logger.error("IO direction does not allow low() operation")

    def high(self):
        if self._dir == GPIO.OUT:
            GPIO.output(self._pin, GPIO.HIGH)
        else:
            logger.error("IO direction does not allow high() operation")
