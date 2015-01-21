import math
import wpilib
try:
    import numpy as np
    M = np.array(
        [[0.5, 0.0],
         [-1.0, -1.0 / math.sqrt(3)],
         [-1.0, 1.0 / math.sqrt(3)]])
except ImportError:
    print("no numpy, hope you aren't trying to use kiwidrive")


def get_wheel_magnitudes(v, m=M):
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
        self.m = np.copy(M)
        self.m[0][0] = 1.5
        self.motor_bias = 0.8;
        self.pid_correction = 0.0
        self.pidcontroller = wpilib.PIDController(
            0.001,
            0.001,
            0.001,
            1,
            lambda: self.gyro.pidGet(),
            lambda output: self.pidWrite(output),
            0.5,
        )
        self.pidcontroller.setSetpoint(0.0)
        self.pidcontroller.setAbsoluteTolerance(2)
        self.pidcontroller.enable()


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
        motor_values = get_wheel_magnitudes(xy, self.m)
        vals = []
        for i, motor in enumerate(self.motors):
            val = motor_values[i] * self.tweaks[i]
            if val < 0:
                val *= self.motor_bias
            val += self.pid_correction
            vals.append(val)
            motor.set(val)
    def pidWrite(self, output):
        print ("pid output: ", output)
        self.pid_correction = output
