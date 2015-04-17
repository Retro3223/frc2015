import wpilib
from strategies import ContainerStrategy


def step(value, min_val):
    """
    Returns "value" unless its less than "min", then it returns 0
    """
    if abs(value) < min_val:
        value = 0
    return value


def step_range(value, min_val, max_val, default):
    """
    Returns "value" unless its out of range
    (not between "min_val" and "max_val"),
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
        self.arm_power = 0  # Smooth(0.0, 0.1)

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

        self.brake_solenoid1 = wpilib.Solenoid(4)
        self.brake_solenoid2 = wpilib.Solenoid(5)

        self.claw_state = True
        self._claw_toggle = False

        # Initialize the limit switches
        self.left_limit_switch = wpilib.DigitalInput(6)
        self.right_limit_switch = wpilib.DigitalInput(4)
        # wpilib.DigitalInput(6)

        # Initialize the compressor watchdog
        self.dog = wpilib.MotorSafety()
        # self.dog.setExpiration(1.75)
        self.dog.setSafetyEnabled(False)

        self.strategies = {}
        ContainerStrategy(self, True)
        ContainerStrategy(self, False)
        ContainerStrategy(self, True, drop=False)
        ContainerStrategy(self, False, drop=False)
        # Select which autonomous mode:
        # * "container-overwhite-drop"
        # * "container-nowhite-drop"
        # * "container-overwhite-nodrop"
        # * "container-nowhite-nodrop"
        self.chooser = wpilib.SendableChooser()
        for auto_mode in self.strategies.keys():
            self.chooser.addObject(auto_mode, auto_mode)
        wpilib.SmartDashboard.putData("Choice", self.chooser)
        self.auto_mode = None

    # Autonomous Mode
    def autonomousInit(self):
        self.auto_mode = self.chooser.getSelected()
        assert self.auto_mode in self.strategies
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
        self.raising_winch = False
        self.minning_winch = False
        self.compressor.start()

    def brake_on(self):
        self.brake_solenoid1.set(True)
        self.brake_solenoid2.set(False)

    def brake_off(self):
        self.brake_solenoid1.set(False)
        self.brake_solenoid2.set(True)

    def teleopPeriodic(self):
        # If left trigger pulled, run brake algorithm,
        # otherwise use joystick values to drive
        if self.left_joystick.getRawButton(1):
            self.brake_off()
        else:
            self.brake_on()
        left_wheel, right_wheel = self.drive_values()

        # Feed joystick values into drive system
        self.robotdrive.tankDrive(left_wheel, right_wheel)

        self.do_winch()
        self.trevor_arm()

        # self.claw_notoggle(default_up=True)
        self.claw_toggle()

        # If the right joystick slider is down, also run test mode
        if self.right_joystick.getRawAxis(2) > .5:
            self.test_mode()

    def do_winch(self):
        # Raise winch subroutine
        if self.right_joystick.getRawButton(5):
            self.raising_winch = True
        if self.right_joystick.getRawButton(4):
            self.minning_winch = True
        # Keeps raising winch while other teleop occurs
        if self.raising_winch:
            if self.get_winch_revs() < 328:
                self.winch_set(1.4)
            else:
                self.raising_winch = False
        elif self.minning_winch:
            if self.get_winch_revs() > self.winch_encoder_min():
                self.winch_set(-1.4)
            else:
                self.minning_winch = False
        else:
            # Feed winch controller raw values from the joystick
            # Right joystick button 3 raises winch, button 2 lowers winch
            winch_signal = 1.8 * self.right_joystick.getRawButton(3) + \
                -self.right_joystick.getRawButton(2)
            # Right joystick button 6 overrides encoder,
            # button 7 resets encoder
            self.winch_set(winch_signal)

    def trevor_arm(self):
        """
        Feed arm controller raw values from the joystick
        Left joystick button 3 goes forward, 2 goes backward
        """
        arm_signal = self.left_joystick.getRawButton(3) + \
            -self.left_joystick.getRawButton(2)
        val = 0.9 * arm_signal  # self.arm_power.set(0.9 * arm_signal)
        self.arm_motor.set(-0.5 * val)

    def claw_notoggle(self, default_up=True):
        """
        Handle piston in and out
        Claw is in default state unless Right joystick trigger button pressed
        """
        if default_up:
            pressed_action = self.claw_up
            default_action = self.claw_down
        else:
            pressed_action = self.claw_down
            default_action = self.claw_up

        if self.right_joystick.getRawButton(1):
            pressed_action()
        else:
            default_action()
        self.set_claw()

    def claw_toggle(self):
        """
        Handle piston in and out
        Right joystick trigger button toggles claw in or out
        toggle
        """
        if self.right_joystick.getRawButton(1):
            self._claw_toggle = True
        elif self._claw_toggle:
            self._claw_toggle = False
            self.claw_state = not self.claw_state
            self.set_claw()

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
            print('arm power: ', self.arm_power)  # .value)

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
        if abs(left - right) < .3:
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
        return not self.left_limit_switch.get()

    def right_claw_whisker(self):
        return not self.right_limit_switch.get()

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
            if not (self.right_joystick.getRawButton(6)):
                # Stop the winch if it is going out of bounds
                if (((signal > 0.1 and revs >= self.winch_encoder_max()) or
                     (signal < -0.1 and revs <= self.winch_encoder_min()))):
                    signal = 0
            val = 0.5 * signal

        # Sets the winch motor's value
        self.winch_motor.set(val)


if __name__ == "__main__":
    wpilib.run(Robot)
