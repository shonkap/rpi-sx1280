import spidev


class SpiDevice:
    def __init__(self, spi_id, cs_id):
        self._device = spidev.SpiDev()
        self._spi_id = spi_id
        self._cs_id = cs_id

    def __enter__(self):
        self._device.open(self._spi_id, self._cs_id)

        self._device.mode = 0b00
        self._device.max_speed_hz = 18_000_000

        return self._device

    def __exit__(self, exc_type, exc_value, exc_tb):
        self._device.close()
        return self._device
