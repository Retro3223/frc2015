import unittest
import math
from ddt import ddt, data
from .kiwidrive import (
    normalize_joystick_axes,
    get_wheel_magnitudes,
    KiwiDrive,
)


@ddt
class KiwidriveTests(unittest.TestCase):
    @data((1.0, 1.0, 1.0),
          (0.0, 1.0, 1.0),
          (1.0, 0.0, 1.0),
          (1.0, 0.24, 1.0),
          (0.5, 0.0, 0.5),
          (0.0, 0.5, 0.5),
          (0.5, 0.5, 0.5),
          (-0.5, -1.0, 1.0),
          (-1.0, -1.0, 1.0),
          (0.0, 0.0, 0.0))
    def test_normalize_joystick_axes_magnitudes(self, data):
        (input_x, input_y, expected_magnitude) = data
        (x, y) = normalize_joystick_axes(input_x, input_y)
        self.assertAlmostEqual(
            expected_magnitude,
            math.hypot(x, y),
            places=3)

    @data((1.0, 1.0, 1/math.sqrt(2), 1/math.sqrt(2)),
          (-1.0, -1.0, -1/math.sqrt(2), -1/math.sqrt(2)),
          (1.0, -1.0, 1/math.sqrt(2), -1/math.sqrt(2)),
          (-1.0, 1.0, -1/math.sqrt(2), 1/math.sqrt(2)),
          (0.5, 0.0, 0.5, 0.0),
          (0.5, 1.0, 0.5/math.sqrt(1.25), 1/math.sqrt(1.25)),
          (-0.5, -1.0, -0.5/math.sqrt(1.25), -1/math.sqrt(1.25)),
          (0.0, 0.0, 0.0, 0.0))
    def test_normalize_joystick_axes_values(self, data):
        (input_x, input_y, expected_x, expected_y) = data
        (x, y) = normalize_joystick_axes(input_x, input_y)
        self.assertAlmostEqual(expected_x, x, places=3)
        self.assertAlmostEqual(expected_y, y, places=3)

    @data((1.0, 0.0, 0.5, -1.0, -1.0),
          (-1.0, 0.0, -0.5, 1.0, 1.0),
          (0.0, 1.0, 0.0, -0.5773, 0.5773),
          (0.0, -1.0, 0.0, 0.5773, -0.5773),
          (0.0, 0.0, 0.0, 0.0, 0.0))
    def test_wheel_magnitidues(self, data):
        (input_x, input_y, _, _, _) = data
        expected_magnitudes = data[2:]
        magnitudes = get_wheel_magnitudes((input_x, input_y))
        for i in range(3):
            self.assertAlmostEqual(
                expected_magnitudes[i],
                magnitudes[i],
                places=3)

