import board
import adafruit_pixelbuf
from adafruit_raspberry_pi5_neopixel_write import neopixel_write


class Pi5Pixelbuf(adafruit_pixelbuf.PixelBuf):
    def __init__(self, pin, size):
        self._pin = pin
        super().__init__(size=size, byteorder="GRBW", auto_write=True)

    def _transmit(self, buf):
        neopixel_write(self._pin, buf)


class LEDController:

    def __init__(self):
        self.pixels = Pi5Pixelbuf(board.D18, 8)
        self.current_mode = None
        self.blink_state = False

    def set_mode(self, mode):
        if mode == self.current_mode:
            return

        self.current_mode = mode

        if mode == "INIT":
            self.pixels.fill((128, 255, 0, 00))  # orange

        elif mode == "MANUAL":
            self.pixels.fill((0, 0, 255, 0))  # Blau

        elif mode == "AUTONOMOUS":
            self.pixels.fill((0, 255, 0, 0))  # Grün

        elif mode == "EMERGENCY":
            self.pixels.fill((255, 0, 0, 0))  # Rot

    def blink_red(self):
        self.blink_state = not self.blink_state
        if self.blink_state:
            self.pixels.fill((255, 0, 0, 0))
        else:
            self.pixels.fill((0, 0, 0, 0))

    def off(self):
        self.pixels.fill((0, 0, 0, 0))
