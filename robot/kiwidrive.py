import numpy as np
import math
import wpilib


m = np.array(
    [[1.5, 0.0],
     [-1.0, -1.0 / math.sqrt(3)],
     [-1.0, 1.0 / math.sqrt(3)]])


def get_wheel_magnitudes(v):
    """
    Calculate the magnitudes to drive wheels 1, 2, and 3
    to drive the robot in the direction defined by normalized
    vector v=[x,y]
    """
    return np.dot(m, v)


def normalize_joystick_axes(x, y):
    """
    A joystick axis returns a value in the range [-1.0 .. 1.0]
    Then two joystick axes (x direction, y direction) give us a
    "unit square". We want a unit circle - i.e. the angle is preserved,
    but the magnitude is the same for any angle.
    Return (x, y) the scaled x, y components
    """
    magnitude = math.hypot(x, y)
    side = max(abs(x), abs(y))
    if magnitude == 0.0:
        return (0.0, 0.0)
    return (x * side / magnitude, y * side / magnitude)


class KiwiDrive:
    def __init__(self, joystick, motors):
        self.joystick = joystick
        assert len(motors) == 3
        self.motors = motors
        self.tweaks = [1, 1, 1]
        self.gyro = wpilib.Gyro(0)

    def Drive(self):
        x = self.joystick.getRawAxis(4)
        if abs(x) < 0.2:
            x = 0
        y = self.joystick.getRawAxis(1)
        if abs(y) < 0.2:
            y = 0
        self.RawDrive(
            0.45 * x,
            0.45 * y)

    def RawDrive(self, x, y):
        xy = normalize_joystick_axes(x, y)
        motor_values = get_wheel_magnitudes(xy)
        vals = []
        for i, motor in enumerate(self.motors):
            val = motor_values[i] * self.tweaks[i]
            if val < 0:
               val *= 0.8
            vals.append(val)
            motor.set(val)
        print ("x: %s" % x)
        print ("y: %s" % y)
        print ("xy: %s" % (xy,))
        print ("motor values:: %s" % (vals,))
        print ("rot :: %s" % (self.gyro.getRate(),))
        print ("")
