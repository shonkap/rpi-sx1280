import RPi.GPIO as GPIO


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
        return GPIO.input(self._pin)

    def write(self, state):
        GPIO.output(self._pin, state)

    def low(self):
        GPIO.output(self._pin, GPIO.LOW)

    def high(self):
        GPIO.output(self._pin, GPIO.HIGH)
