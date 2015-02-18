import wpilib
import math
from tiredrive.strategies import Auto3StraightStrategy, TurnStrategy, ContainerStrategy


def step(value, min_val):
    """
    Returns "value" unless its less than "min", then it returns 0
    """
    if abs(value) < min_val:
        value = 0
    return value


def step_range(value, min_val, max_val, default):
    """
    Returns "value" unless its out of range (not between "min_val" and "max_val"),
    then it returns "default
    """
    if not (min_val <= value <= max_val):
        value = default
    return value


class Smooth:
    """
    # Class to maintain state for slow start up and slow down of motors
    # to reduce jerkiness
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


class Robot(wpilib.IterativeRobot):
    """
    Robot object definition
    """
    def robotInit(self):
        """
        Initialize all of the sensors and controllers on the robot
        """

        # Initialize the Joysticks
        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)

        # Initialize the drive motors
        self.left_motor = wpilib.Talon(0)
        self.right_motor = wpilib.Talon(1)

        self.left_encoder = wpilib.Encoder(5, 7)
        self.right_encoder = wpilib.Encoder(8, 9)

        # Initialize the drive system
        self.robotdrive = wpilib.RobotDrive(self.left_motor, self.right_motor)

        # Initialize the winch motor
        self.winch_motor = wpilib.Talon(2)

        # Initialize the arm motor
        self.arm_motor = wpilib.Talon(3)
        self.arm_power = Smooth(0.0, 0.05)

        # Initialize the accelerometer
        self.accel = wpilib.BuiltInAccelerometer()
        self.a_x_sum = 0.0
        self.a_x_count = 0
        self.a_y_sum = 0.0
        self.a_y_count = 0

        # Initialize the gyro
        self.gyro = wpilib.Gyro(0)

        # Initialize the winch encoder
        self.winch_encoder = wpilib.Encoder(1, 2)
        self._winch_encoder_min = 8

        self.last_winch_signal = 0

        # Initialize the compressor
        self.compressor = wpilib.Compressor(0)

        # Initialize the pneumatic solenoids for the claw
        self.solenoid1 = wpilib.Solenoid(1)
        self.solenoid2 = wpilib.Solenoid(2)

        self.claw_state = True
        self.claw_toggle = False

        # Initialize the ultrasonic sensors
        self.left_ultrasonic_sensor = wpilib.AnalogInput(1)
        self.right_ultrasonic_sensor = wpilib.AnalogInput(2)

        # Initialize the optical sensors
        self.left_optical_sensor = wpilib.DigitalInput(3)
        self.right_optical_sensor = wpilib.DigitalInput(4)

        # Initialize the limit switches
        self.left_limit_switch = wpilib.DigitalInput(6)
        self.right_limit_switch = self.left_limit_switch
        # wpilib.DigitalInput(6)

        # Initialize the compressor watchdog
        self.dog = wpilib.MotorSafety()
        # self.dog.setExpiration(1.75)
        self.dog.setSafetyEnabled(False)

        self.strategies = {}
        Auto3StraightStrategy(self)
        TurnStrategy(self)
        ContainerStrategy(self)
        # Select which autonomous mode: "tote", "container", "tripletote"
        self.auto_mode = "3-tote-straight"

    # Autonomous Mode
    def autonomousInit(self):
        assert self.auto_mode in [
            'container',
            "tote",
            '3-tote-straight',
        ]
        self.compressor.start()
        self.winch_setpoint_zero = self.winch_setpoint = self.get_winch_revs()

        self.auto_state = "start"
        self.strategies[self.auto_mode].autonomousInit()

    def autonomousPeriodic(self):
        """
        # Runs an autonomous mode method based on the selected mode
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
        self.left_motor.set(-val)
        self.right_motor.set(val)

    def pivot_clockwise(self, val):
        self.left_motor.set(-val)
        self.right_motor.set(-val)

    # Teleop Mode
    def teleopInit(self):
        self.winch_setpoint = self.get_winch_revs()
        self.reset_auto()
        self.raising_winch = False

        self.compressor.start()

    def teleopPeriodic(self):
        # Override everything (prevents all teleop) and run
        # autonomously to pick up tote
        # Press and hold button 6
        if self.left_joystick.getRawButton(6):
            self.auto_pickup()
            return
        # Pressing button 7 will reset auto_pickup's process
        if self.auto_state == "finished" or self.left_joystick.getRawButton(7):
            self.reset_auto()

        # If left trigger pulled, run brake algorithm,
        # otherwise use joystick values to drive
        if self.left_joystick.getRawButton(1):
            rotation_values = self.brake_rotation()
            linear_values = self.brake_linear()
            left_wheel = rotation_values[0] + linear_values[0]
            right_wheel = rotation_values[1] + linear_values[1]
        else:
            left_wheel, right_wheel = self.drive_values()

        # Feed joystick values into drive system
        self.robotdrive.tankDrive(left_wheel, right_wheel)

        # Raise winch subroutine
        if self.right_joystick.getRawButton(5):
            self.raising_winch = True
        if self.right_joystick.getRawButton(4):
            self.raising_winch = False
        # Keeps raising winch while other teleop occurs
        if self.raising_winch:
            if self.get_winch_revs() < 500:
                self.winch_motor.set(.5)
            else:
                self.raising_winch = False
        else:
            # Feed winch controller raw values from the joystick
            # Right joystick button 3 raises winch, button 2 lowers winch
            winch_signal = self.right_joystick.getRawButton(3) + \
                -self.right_joystick.getRawButton(2)
            # Right joystick button 6 overrides encoder,
            # button 7 resets encoder
            self.winch_set(winch_signal)

        # Feed arm controller raw values from the joystick
        # Left joystick button 3 goes forward, 2 goes backward
        arm_signal = self.left_joystick.getRawButton(3) + \
            -self.left_joystick.getRawButton(2)
        val = self.arm_power.set(arm_signal)
        self.arm_motor.set(val)

        # Handle piston in and out
        # Right joystick trigger button toggles claw in or out
        if self.right_joystick.getRawButton(1):
            self.claw_toggle = True
        elif self.claw_toggle:
            self.claw_toggle = False
            self.claw_state = not self.claw_state
            self.set_claw()

        # If the right joystick slider is down, also run test mode
        if self.right_joystick.getRawAxis(2) > .5:
            self.test_mode()

    # Robotpy's broken test mode
    def testPeriodic(self):
        pass

    def test_mode(self):
        """
        # Test Mode
        # calculates and prints values to be used in testing
        """

        # Calculate x and y distance travelled using accelerometer
        a_x = self.accel.getX()
        self.a_x_sum += a_x
        self.a_x_count += 1
        a_y = self.accel.getY()
        self.a_y_sum += a_y
        self.a_y_count += 1

        # Prints acceleration values when right button 8 is pressed
        if self.right_joystick.getRawButton(8):
            print('x sum: ', self.a_x_sum, ' x count: ', self.a_x_count)
            print('y sum: ', self.a_y_sum, ' y count: ', self.a_y_count)

        # Resets accelerometer counts when right button 9 is pressed
        if self.right_joystick.getRawButton(9):
            self.a_x_sum = 0.0
            self.a_x_count = 0
            self.a_y_sum = 0.0
            self.a_y_count = 0

        # Prints limit switch sensor values when left button 6 is pressed
        if self.left_joystick.getRawButton(4):
            print("left limit switch: ", self.left_claw_whisker())
            print("right limit switch: ", self.right_claw_whisker())

        # Prints ultrasonic sensor values when left button 10 is pressed
        if self.left_joystick.getRawButton(10):
            print("left_ultrasonic_sensor: ",
                  self.left_ultrasonic_sensor.getValue())
            print("right_ultrasonic_sensor: ",
                  self.right_ultrasonic_sensor.getValue())

        # Prints optical sensor values when left button 11 is pressed
        if self.left_joystick.getRawButton(11):
            print("left encoder: ", self.left_encoder.get())
            print("right encoder: ", self.right_encoder.get())

        # Print current winch encoder value if right button 6 is pressed
        # This button also overrides winch encoder safety bounds
        if self.right_joystick.getRawButton(6):
            revs = self.get_winch_revs()
            print('winch revolutions: ', revs)

        # Print current gyro value if left button 8 is pressed
        if self.left_joystick.getRawButton(8):
            angle = self.gyro.getAngle()
            print('angle: ', angle)

        # Reset gyro to 0 if left button 9 is pressed
        if self.left_joystick.getRawButton(9):
            self.gyro.reset()

        if self.left_joystick.getRawButton(2) or \
                self.left_joystick.getRawButton(3):
            print('arm power: ', self.arm_power.value)

    # Disabled Mode
    def disabledPeriodic(self):
        self.compressor.stop()

    # Helper Functions

    def drive_values(self):
        """
        # Set "left" and "right" variables to the left and right
        # joystick outputs provided they are more than "joystick_threshold"
        """
        joystick_threshold = 0.2
        left = step(
            self.left_joystick.getRawAxis(1),
            joystick_threshold,
        )
        right = step(
            self.right_joystick.getRawAxis(1),
            -joystick_threshold,
        )

        # Fuzzy match where if the left and right joysticks are moved
        # about the same, then it moves the tankDrive the average of
        # the two values
        if abs(left - right) < .1:
            left = right = (left + right) / 2.0

        return left, right

    def brake_rotation(self):
        """
        Brakes robot if it's rotating
        by powering the motors in the direction opposite the rotation
        """
        gyro_rate = self.gyro.getRate()
        wheel_rotation = gyro_rate * .1
        return wheel_rotation, -wheel_rotation

    def brake_linear(self):
        """
        Brakes robot if it's moving forward or backward
        by powering the motors in the direction opposite the movement
        """
        accel_y = self.accel.getY()
        wheel_motion = -accel_y * .1
        return wheel_motion, wheel_motion

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

    def left_claw_whisker(self):
        return self.left_limit_switch.get()

    def right_claw_whisker(self):
        return self.right_limit_switch.get()

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
        if self.right_joystick.getRawButton(7):
            self.winch_encoder.reset()

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
            if not (self.right_joystick.getRawButton(6)):
                # Stop the winch if it is going out of bounds
                if (((signal > 0.1 and revs >= self.winch_encoder_max()) or
                     (signal < -0.1 and revs <= self.winch_encoder_min()))):
                    signal = 0
            val = 0.5 * signal

        # Sets the winch motor's value
        self.winch_motor.set(val)

    def reset_auto(self):
        self.auto_state = "start"

    def auto_pickup(self):
        # state "start": pull in solenoid
        if self.auto_state == "start":
            self.claw_down()
            self.set_claw()
            self.auto_state = "drive"

        # state "drive": drive forward
        if self.auto_state == "drive":
            if not self.left_claw_whisker() and not self.right_claw_whisker():
                self.forward(0.5)
            else:
                self.auto_state = "pickup"

        # state "pickup": lift up to pick up container
        if self.auto_state == "pickup":
            if self.get_winch_revs() < 500:
                self.winch_motor.set(.5)
            else:
                self.auto_state = "clawout"

        # state "clawout": push out solenoid
        if self.auto_state == "clawout":
            self.claw_up()
            self.set_claw()
            self.auto_state = "finished"


if __name__ == "__main__":
    wpilib.run(Robot)
