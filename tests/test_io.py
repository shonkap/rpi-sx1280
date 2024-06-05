import spidev
import RPi.GPIO as GPIO

from loguru import logger
from rpi_sx1280.io import IOPin, SpiDevice


def test_io_pin_read(mocker):
    mocker.patch.object(GPIO, "setup", autospec=True)
    mocker.patch.object(GPIO, "input", autospec=True)

    _pin = 0
    _dir = GPIO.IN

    io = IOPin(_pin, _dir)

    GPIO.setup.assert_called_once_with(_pin, _dir)
    GPIO.input.assert_called_once_with(_pin)

    io.read()

    assert GPIO.input.call_count == 2


def test_io_pin_high(mocker):
    mocker.patch.object(GPIO, "setup", autospec=True)
    mocker.patch.object(GPIO, "output", autospec=True)

    _pin = 0
    _dir = GPIO.OUT
    _init = GPIO.LOW

    io = IOPin(_pin, _dir, init=_init)

    GPIO.setup.assert_called_once_with(_pin, _dir, initial=_init)
    GPIO.input.return_value = GPIO.HIGH

    assert io.read() == GPIO.HIGH

    GPIO.input.assert_called_once_with(_pin)


def test_io_pin_low(mocker):
    mocker.patch.object(GPIO, "setup", autospec=True)
    mocker.patch.object(GPIO, "output", autospec=True)

    _pin = 0
    _dir = GPIO.OUT
    _init = GPIO.LOW

    io = IOPin(_pin, _dir, init=_init)

    GPIO.setup.assert_called_once_with(_pin, _dir, initial=_init)

    io.low()

    GPIO.output.assert_called_once_with(_pin, GPIO.LOW)


def test_io_pin_high(mocker):
    mocker.patch.object(GPIO, "setup", autospec=True)
    mocker.patch.object(GPIO, "output", autospec=True)

    _pin = 0
    _dir = GPIO.OUT
    _init = GPIO.LOW

    io = IOPin(_pin, _dir, init=_init)

    GPIO.setup.assert_called_once_with(_pin, _dir, initial=_init)

    io.high()

    GPIO.output.assert_called_once_with(_pin, GPIO.HIGH)


def test_io_pin_write_high(mocker):
    mocker.patch.object(GPIO, "setup", autospec=True)
    mocker.patch.object(GPIO, "output", autospec=True)

    _pin = 0
    _dir = GPIO.OUT
    _init = GPIO.LOW

    io = IOPin(_pin, _dir, init=_init)

    GPIO.setup.assert_called_once_with(_pin, _dir, initial=_init)

    io.write(GPIO.HIGH)

    GPIO.output.assert_called_once_with(_pin, GPIO.HIGH)


def test_io_pin_write_low(mocker):
    mocker.patch.object(GPIO, "setup", autospec=True)
    mocker.patch.object(GPIO, "output", autospec=True)

    _pin = 0
    _dir = GPIO.OUT
    _init = GPIO.LOW

    io = IOPin(_pin, _dir, init=_init)

    GPIO.setup.assert_called_once_with(_pin, _dir, initial=_init)

    io.write(GPIO.LOW)

    GPIO.output.assert_called_once_with(_pin, GPIO.LOW)


def test_spidevice_use_with(mocker):
    mocker.patch.object(spidev, "SpiDev", autospec=True)

    _spi_id = 0
    _cs_id = 0

    _spi_dev = SpiDevice(_spi_id, _cs_id)

    with _spi_dev as spi:
        assert _spi_dev._device.open.call_count == 1

        assert spi.mode == 0b00
        assert spi.max_speed_hz == 18_000_000

    assert _spi_dev._device.close.call_count == 1
