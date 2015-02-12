import math
import wpilib
from xbox import XboxController
try:
    import numpy as np
    M = np.array(
        [[-0.5, 0.0],
         [1.0, -1.0 / math.sqrt(3)],
         [1.0, 1.0 / math.sqrt(3)]])
except ImportError:
    print("no numpy, hope you aren't trying to use kiwidrive")


def get_wheel_magnitudes(v, m=None):
    """
    Calculate the magnitudes to drive wheels 1, 2, and 3
    to drive the robot in the direction defined by normalized
    vector v=[x,y]
    """
    if m is None:
        m = M
    return np.dot(m, v)
def get_wheel_magnitudes2(x, y):
	result = [0,0,0]
	result[0] = -1.28 * x;
	result[1] = 0.8 * x + -0.75 * y
	result[2] = 0.8 * x +  0.75 * y
	return result


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
        self.xbox = XboxController(self.joystick)
        assert len(motors) == 3
        self.motors = motors
        self.tweaks = [1, 1, 1]
        self.gyro = wpilib.Gyro(0)
        self.m = np.copy(M)
        self.m[0][0] = -1.6
        self.m[:, 1] *= 1.3 # make forward a bit faster
        self.m[:, 0] *= 0.8 # make strafe a bit slower
        self.motor_bias = 0.8;
        self.pid_correction = 0.0
        self.last_rot = 0.0
        self.last_angle = 0
        self.last_angle_count = 0
        self.waiting_to_reenable = False
        self.pidcontroller = wpilib.PIDController(
            0.015,
            0.0,
            0.0,
            .1,
            lambda: self.getAngle(),
            lambda output: self.pidWrite(output),
        )
        
        self.pidcontroller.setAbsoluteTolerance(5)
        
    def Enable(self):
        self.pidcontroller.setSetpoint(self.getAngle())
        self.pidcontroller.enable()

    def Disable(self):
        self.pidcontroller.disable()

    def getAngle(self):
        return int(self.gyro.pidGet())
    
    def Drive(self):
        x = self.joystick.getRawAxis(4)
        if abs(x) < 0.2:
            x = 0
        y = self.joystick.getRawAxis(1)
        if abs(y) < 0.2:
            y = 0
        # rot is +1.0 for right trigger, -1.0 for left
        rot = self.xbox.right_trigger() + -self.xbox.left_trigger();
        self.RawDrive(
            x,
            y,
            rot)

    def RawDrive(self, x, y, rot):
        xy = normalize_joystick_axes(x, y)
        motor_values = get_wheel_magnitudes(xy, self.m)
        vals = []
        if rot != 0:
            self.pidcontroller.reset()
        if rot == 0 and self.last_rot != 0:
            self.waiting_to_reenable = True
            print ("WAITING TO REENABLE")
        elif self.waiting_to_reenable:
            if self.last_angle == self.getAngle():
                self.last_angle_count += 1
            else:
                self.last_angle_count = 0

            if self.last_angle_count >= 10:
                self.waiting_to_reenable = False
                self.Enable()
                print ("REENABLING")
        self.last_angle = self.getAngle()
        self.last_rot = rot
        for i, motor in enumerate(self.motors):
            val = motor_values[i] * self.tweaks[i]
            val += rot * .3
            if val < 0:
                val *= self.motor_bias
            val += self.pid_correction
            vals.append(val)
            motor.set(val)

    def pidWrite(self, output):
        print ("pid output: ", output)
        self.pid_correction = output
