import pytest
import math
from kiwidrive.kiwi import (
    normalize_joystick_axes,
    get_wheel_magnitudes,
)


joystick_magnitude_data = [(1.0, 1.0, 1.0),
                           (0.0, 1.0, 1.0),
                           (1.0, 0.0, 1.0),
                           (1.0, 0.24, 1.0),
                           (0.5, 0.0, 0.5),
                           (0.0, 0.5, 0.5),
                           (0.5, 0.5, 0.5),
                           (-0.5, -1.0, 1.0),
                           (-1.0, -1.0, 1.0),
                           (0.0, 0.0, 0.0)]


@pytest.mark.parametrize("input_x,input_y,expected_magnitude",
                         joystick_magnitude_data)
def test_joystick_axes_magnitudes(input_x,
                                  input_y,
                                  expected_magnitude):
    (x, y) = normalize_joystick_axes(input_x, input_y)
    assert round(expected_magnitude - math.hypot(x, y), 3) == 0


joystick_axes_data = [(1.0, 1.0, 1/math.sqrt(2), 1/math.sqrt(2)),
                      (-1.0, -1.0, -1/math.sqrt(2), -1/math.sqrt(2)),
                      (1.0, -1.0, 1/math.sqrt(2), -1/math.sqrt(2)),
                      (-1.0, 1.0, -1/math.sqrt(2), 1/math.sqrt(2)),
                      (0.5, 0.0, 0.5, 0.0),
                      (0.5, 1.0, 0.5/math.sqrt(1.25), 1/math.sqrt(1.25)),
                      (-0.5, -1.0, -0.5/math.sqrt(1.25), -1/math.sqrt(1.25)),
                      (0.0, 0.0, 0.0, 0.0)]


@pytest.mark.parametrize("input_x,input_y,expected_x,expected_y",
                         joystick_axes_data)
def test_joystick_axes_values(input_x, input_y, expected_x, expected_y):
    (x, y) = normalize_joystick_axes(input_x, input_y)
    assert round(expected_x - x, 3) == 0
    assert round(expected_y - y, 3) == 0


wheel_magnitude_data = [(1.0, 0.0, -1.6, 1.0, 1.0),
                        (-1.0, 0.0, 1.6, -1.0, -1.0),
                        (0.0, 1.0, 0.0, -0.5773, 0.5773),
                        (0.0, -1.0, 0.0, 0.5773, -0.5773),
                        (0.0, 0.0, 0.0, 0.0, 0.0)]


@pytest.mark.parametrize("input_x,input_y,exp_0,exp_1,exp_2",
                         wheel_magnitude_data)
def test_wheel_magnitidues(input_x, input_y, exp_0, exp_1, exp_2):
    magnitudes = get_wheel_magnitudes((input_x, input_y), 0)
    assert round(exp_0 - magnitudes[0], 3) == 0
    assert round(exp_1 - magnitudes[1], 3) == 0
    assert round(exp_2 - magnitudes[2], 3) == 0
