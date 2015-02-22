import math
import wpilib
import xbox as joy
import strategies as strats

try:
    import numpy as np
    M = np.array(
        [[-1.6,  0.0],
         [+1.0, -1.0 / math.sqrt(3)],
         [+1.0,  1.0 / math.sqrt(3)]])
except ImportError:
    print("no numpy; hope you aren't trying to use kiwidrive")


def get_wheel_magnitudes(v, m=None):
    """
    Calculate the magnitudes to drive wheels 1, 2, and 3
    to drive the robot in the direction defined by normalized
    vector v=[x,y]
    """
    if m is None:
        m = M
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
    def __init__(self, joystick, motors):
        """
        Initialize all of the sensors and controllers on the robot
        """

        # Initialize the Joystick
        self.joy = joy.XboxController(joystick)

        # Initialize the drive motors
        assert len(motors) == 3
        self.motors = motors

        self.tweaks = [1, 1, 1]

        # modify values for better driving
        self.m = np.copy(M)
        self.m[:, 1] *= 1.3  # make forward a bit faster
        self.m[:, 0] *= 0.8  # make strafe a bit slower
        self.motor_bias = 0.8

        # Initialize the arm motor
        self.arm_motor = wpilib.Talon(4)
        self.arm_power = Smooth(0.0, 0.05)

        # Initialize the winch motor
        self.winch_motor = wpilib.Talon(3)

        # Initialize the winch encoder
        self.winch_encoder = wpilib.Encoder(1, 2)
        self._winch_encoder_min = 8

        self.last_winch_signal = 0

        # Initialize the compressor
        self.compressor = wpilib.Compressor(0)

        # Initialize the compressor watchdog
        self.dog = wpilib.MotorSafety()
        # self.dog.setExpiration(1.75)
        self.dog.setSafetyEnabled(False)

        # Initialize the pneumatic solenoids for the claw
        self.solenoid1 = wpilib.Solenoid(1)
        self.solenoid2 = wpilib.Solenoid(2)

        self.claw_state = True
        self.claw_toggle = False

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

        # Initialize autonomous strategies
        self.strategies = {}
        strats.Auto3ToteStrategy(self)
        strats.ContainerStrategy(self, True)
        strats.ContainerStrategy(self, False)

    def autonomousInit(self, auto_mode):
        """
        Runs an autonomous mode method based on the selected mode
        """
        assert auto_mode in self.strategies.keys()
        self.auto_mode = auto_mode
        self.winch_setpoint_zero = self.winch_setpoint = self.get_winch_revs()
        self.strategies[self.auto_mode].autonomousInit()

        self.compressor.start()

    def autonomousPeriodic(self):
        """
        Runs an autonomous mode method based on the selected mode
        """
        self.dog.feed()
        self.strategies[self.auto_mode].autonomousPeriodic()

    def maintain_claw(self):
        while True:
            self.set_claw()
            yield

    def maintain_winch(self):
        while True:
            self.winch_set(0)
            yield

    def get_winch_revs(self):
        return -self.winch_encoder.get()

    def winch_encoder_min(self):
        return self._winch_encoder_min

    def winch_encoder_max(self):
        return self._winch_encoder_min + 1162

    def forward(self, val):
        self.RawDrive(0, val, 0)

    def pivot_clockwise(self, val):
        self.RawDrive(0, 0, val)

    def Enable(self):
        self.pidcontroller.setSetpoint(self.getAngle())
        self.pidcontroller.enable()
        self.winch_setpoint = self.get_winch_revs()

        self.compressor.start()

    def Disable(self):
        self.pidcontroller.disable()
        self.compressor.stop()

    def getAngle(self):
        return int(self.gyro.pidGet())

    def Drive(self):
        x = self.joy.analog_drive_x()
        y = self.joy.analog_drive_y()
        # rot is +1.0 for right trigger, -1.0 for left
        rot = self.joy.analog_rot()
        self.RawDrive(x, y, rot)

        # Feed winch controller raw values from the joystick
        winch_signal = self.joy.analog_winch()
        # Right joystick button 6 overrides encoder,
        # button 7 resets encoder
        self.winch_set(winch_signal)

        # Feed arm controller raw values from the joystick
        # Left joystick button 3 goes forward, 2 goes backward
        arm_signal = self.joy.analog_arm()
        self.arm_motor.set(self.arm_power.set(arm_signal * .3))

        # Handle piston in and out
        # Right joystick trigger button toggles claw in or out
        if self.joy.digital_claw():
            self.claw_toggle = True
        elif self.claw_toggle:
            self.claw_toggle = False
            self.claw_state = not self.claw_state
            self.set_claw()

        # If the right joystick slider is down, also run test mode
        if self.joy.digital_test():
            self.test_mode()

    def test_mode(self):
        """
        # Test Mode
        # calculates and prints values to be used in testing
        """
        print('legalize crystal fucking weed')

        print('winch revolutions: ', self.get_winch_revs())

        print('angle: ', self.gyro.getAngle())

    def RawDrive(self, x, y, rot):
        xy = normalize_joystick_axes(x, y)
        motor_values = get_wheel_magnitudes(xy, self.m)

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

    def pidWrite(self, output):
        print("pid output: ", output)
        self.pid_correction = output

    def brake_rotation(self):
        """
        Brakes robot if it's rotating
        by powering the motors in the direction opposite the rotation
        """
        gyro_rate = self.gyro.getRate()
        return self.RawDrive(0, 0, -gyro_rate * .1)

    def brake_linear(self):
        """
        Brakes robot if it's moving forward or backward
        by powering the motors in the direction opposite the movement
        """
        accel_y = self.accel.getY()
        return self.RawDrive(0, -accel_y * .1, 0)

    def set_claw(self):
        """
        # Moves claw into "claw_state" position
        """
        self.solenoid1.set(not self.claw_state)
        self.solenoid2.set(self.claw_state)

    def claw_up(self):
        """
        # Pushes claw out
        """
        self.claw_state = False

    def claw_down(self):
        """
        # Pulls claw in
        """
        self.claw_state = True

    def winch_set(self, signal):
        """
        Set winch controller safely by taking max and min encoder values
        into account, unless you're pressing the override button
        (right joystick, button 6)

        signal=0 -> maintain winch position
        signal>0 -> winch up?
        signal<0 -> winch down?
        """

        # Reset winch encoder value to 0 if right button 7 is pressed
        if self.joy.digital_winch_encoder_reset():
            self.winch_encoder.reset()
            self.winch_setpoint = self.get_winch_revs()

        # Initializes "revs" to the winch encoder's current value
        revs = self.get_winch_revs()

        # Sets "winch_setpoint" when driver takes finger off winch button
        if self.last_winch_signal != 0 and signal == 0:
            self.winch_setpoint = revs
        self.last_winch_signal = signal

        # If no winch signal, maintain winch's height position
        # Else moves winch according to winch signal
        if signal == 0:
            val = 0.1 - 0.01 * (revs - self.winch_setpoint)
        else:
            # Pressing right button 6 overrides winch's safety bounds
            if not (self.joy.digital_winch_override()):
                # Stop the winch if it is going out of bounds
                if (((signal > 0.1 and revs >= self.winch_encoder_max()) or
                     (signal < -0.1 and revs <= self.winch_encoder_min()))):
                    signal = 0
            val = 0.5 * signal

        # Sets the winch motor's value
        self.winch_motor.set(val)
