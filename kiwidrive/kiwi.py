import math
import wpilib
import xbox as joy

try:
    import numpy as np
    M = np.array(
        [[-1.6,  0.0],
         [+1.0, -1.0 / math.sqrt(3)],
         [+1.0,  1.0 / math.sqrt(3)]])
except ImportError:
    print("no numpy; hope you aren't trying to use kiwidrive")


def get_wheel_magnitudes(v, rot_offset, m=None):
    """
    Calculate the magnitudes to drive wheels 1, 2, and 3
    to drive the robot in the direction defined by normalized
    vector v=[x,y]
    """
    rot = math.atan2(v[1], v[0]) + rot_offset
    if m is None:
        m = M
    v2 = (math.cos(rot), math.sin(rot))
    return math.hypot(*v) * np.dot(m, v2)


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
        return 0.0, 0.0
    return x * side / magnitude, y * side / magnitude


def step(value, min_val):
    """
    Returns "value" unless its less than "min", then it returns 0
    """
    if abs(value) < min_val:
        value = 0
    return value


class Smooth:
    """
    Class to maintain state for slow start up and slow down of motors
    to reduce jerkiness
    """
    def __init__(self, val, stp):
        self.value = val
        self.step = stp

    def set(self, new_val):
        if self.value < new_val:
            self.value = min(self.value + self.step, new_val)
        else:
            self.value = max(self.value - self.step, new_val)
        return self.value

    def force(self, new_val):
        self.value = new_val
        return self.value


class KiwiDrive:
    def __init__(self, robot, joy, motors):
        """
        Initialize all of the sensors and controllers on the robot
        """

        # Initialize the Joystick
        self.robot = robot
        self.joy = joy

        # Initialize the drive motors
        assert len(motors) == 3
        self.motors = motors

        self.tweaks = [1, 1, 1]

        # modify values for better driving
        self.m = np.copy(M)
        self.m[:, 1] *= 1.3  # make forward a bit faster
        self.m[:, 0] *= 0.8  # make strafe a bit slower
        self.motor_bias = 0.8

        # Initialize the compressor watchdog
        self.dog = wpilib.MotorSafety()
        # self.dog.setExpiration(1.75)
        self.dog.setSafetyEnabled(False)

        # Initialize the accelerometer
        self.accel = wpilib.BuiltInAccelerometer()

        # Initialize the gyro
        self.gyro = wpilib.Gyro(0)

        # Initialize the PID Controller
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

    def forward(self, val):
        self.RawDrive(0, val, 0, 0)

    def pivot_clockwise(self, val):
        self.RawDrive(0, 0, 0, val)

    def Enable(self):
        self.pidcontroller.setSetpoint(self.getAngle())
        self.pidcontroller.enable()

    def Disable(self):
        self.pidcontroller.disable()

    def getAngle(self):
        return int(self.gyro.pidGet())

    def Drive_Kiwi(self):
        x = self.joy.analog_drive_x()
        y = self.joy.analog_drive_y()
        # rot is +1.0 for right trigger, -1.0 for left
        rot = self.joy.analog_rot()
        heading_offset = self.joy.d_pad()
        if heading_offset == -1:
            heading_offset = 0
        else:
            heading_offset -= self.getAngle()
            if abs(x) < 0.2 and abs(y) < 0.2:
                # set magnitude to 0.5;
                # heading_offset will change the direction
                x = 0.5
                y = 0
        self.RawDrive(x, y, heading_offset, rot)

    def Drive_Tank(self):
        x = self.joy.analog_drive_x()
        x = step(x, 0.2)
        y = self.joy.analog_drive_y()
        y = step(y, 0.2)
        # rot is +1.0 for right trigger, -1.0 for left
        rot = self.joy.analog_rot()
        self.tank_drive(x, y)

    def RawDrive(self, x, y, heading_offset, rot):
        """
        heading_offset - offset in degrees to the direction we are driving
            the robot
        rot - offset in signal [-1, 1] to all motors (kind of a low-level
            rotational offset)
        """
        xy = normalize_joystick_axes(x, y)
        motor_values = get_wheel_magnitudes(xy, heading_offset, self.m)

        # Deals with rotation and calming down the gyro
        if rot != 0:
            self.pidcontroller.reset()
        if rot == 0 and self.last_rot != 0:
            self.waiting_to_reenable = True
            print("WAITING TO REENABLE")
        elif self.waiting_to_reenable:
            if self.last_angle == self.getAngle():
                self.last_angle_count += 1
            else:
                self.last_angle_count = 0

            if self.last_angle_count >= 10:
                self.waiting_to_reenable = False
                self.Enable()
                print("REENABLING")
        self.last_angle = self.getAngle()
        self.last_rot = rot

        for i, motor in enumerate(self.motors):
            val = motor_values[i] * self.tweaks[i]
            val += rot * .3
            if val < 0:
                val *= self.motor_bias
            val += self.pid_correction
            motor.set(val)

    def tank_drive(self,x,y,):
        self.motors[0].set(0.5*x)
        self.motors[1].set(0.5*x - y)
        self.motors[2].set(0.5*x + y)

    def pidWrite(self, output):
        self.pid_correction = output

    def brake_rotation(self):
        """
        Brakes robot if it's rotating
        by powering the motors in the direction opposite the rotation
        """
        gyro_rate = self.gyro.getRate()
        return self.RawDrive(0, 0, 0, -gyro_rate * .1)

    def brake_linear(self):
        """
        Brakes robot if it's moving forward or backward
        by powering the motors in the direction opposite the movement
        """
        accel_y = self.accel.getY()
        return self.RawDrive(0, -accel_y * .1, 0, 0)
