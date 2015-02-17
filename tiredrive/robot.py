import wpilib
import math


class ParallelGenerators:
    def __init__(self):
        self.generators = {}
        self.afters = {}

    def add(self, name, generator, after=None):
        self.generators[name] = iter(generator)
        if after is not None:
            self.after(name, after)

    def after(self, succeed_name, precede_name):
        if precede_name not in self.afters:
            self.afters[precede_name] = []
        self.afters[precede_name].append(
            (succeed_name, self.generators[succeed_name]))
        del self.generators[succeed_name]

    def cancel(self, name):
        if name in self.afters:
            for (new_name, generator) in self.afters[name]:
                self.generators[new_name] = generator
        del self.generators[name]

    def next(self):
        results = {}
        items = list(self.generators.items())
        for (name, g) in items:
            try:
                val = g.__next__()
                results[name] = val
            except StopIteration:
                self.cancel(name)
        return results


def step(value, min_val):
    """
    Returns "value" unless its less than "min", then it returns 0
    """
    if abs(value) < min_val:
        value = 0
    return value


# Returns "value" unless its out of range (not between "min_val" and
# "max_val"), then it returns "default"
def step_range(value, min_val, max_val, default):
    """
    Returns "value" unless its out of range (not between "min" and "max"),
    then it returns "default
    """
    if not (min_val <= value <= max_val):
        value = default
    return value


# Class to maintain state for slow start up and slow down of motors
# to reduce jerkiness
class Smooth:
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

        self.left_encoder = wpilib.Encoder(6, 7)
        self.right_encoder = wpilib.Encoder(8, 9)

        # Initialize the drive system
        self.robotdrive = wpilib.RobotDrive(self.left_motor, self.right_motor)

        # Initialize the winch motor
        self.winch_motor = wpilib.Talon(2)
        self.winch_power = Smooth(0.0, 0.1)

        # Initialize the arm motor
        self.arm_motor = wpilib.Talon(3)
        self.arm_power = Smooth(0.0, 0.01)

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
        self.left_limit_switch = wpilib.DigitalInput(5)
        self.right_limit_switch = self.left_limit_switch
        # wpilib.DigitalInput(6)

        # Initialize the compressor watchdog
        self.dog = wpilib.MotorSafety()
        # self.dog.setExpiration(1.75)
        self.dog.setSafetyEnabled(False)

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
        self.positioned_count = 0
        if self.auto_mode == "container":
            pass
        elif self.auto_mode == "tote":
            self.auto = iter(self.auto_tote_periodic())
        elif self.auto_mode == '3-tote-straight':
            self.auto = ParallelGenerators()
            self.claw_down()
            self.auto.add("claw", self.maintain_claw())
            self.auto.add("pickup1", self.auto_pickup_tote(1))
            self.auto.add("winch", self.maintain_winch())
            self.auto.after("winch", "pickup1")
            self.auto.add("drive1", self.auto_drive_until_tote(1))
            self.auto.after("drive1", "pickup1")
            self.auto.add("drop1", self.drop_tote())
            self.auto.after("drop1", 'drive1')
            self.auto.add("drive1.5", self.auto_drive_until_liftable(1))
            self.auto.after("drive1.5", "drop1")

    # Runs an autonomous mode method based on the selected mode
    def autonomousPeriodic(self):
        self.dog.feed()
        if self.auto_mode == "tote":
            self.auto.__next__()
        elif self.auto_mode == '3-tote-straight':
            self.auto.next()
        elif self.auto_mode == "container":
            self.auto_container_periodic()
        elif self.auto_mode == "tripletote":
            self.auto_tripletote_periodic()

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

    def auto_pickup_tote(self, tote_height):
        tote_revs = 320
        assert self.get_winch_revs() < 20
        self.winch_setpoint = self.winch_setpoint_zero + \
            tote_revs * tote_height
        durped = False
        while self.get_winch_revs() < self.winch_setpoint:
            if not durped and self.get_winch_revs() >= 70:
                self.claw_up()
                durped = True
            self.winch_set(1.0)
            yield

    def auto_drive_until_tote(self, tote_number):
        revs0 = self.right_encoder.get()
        while True:
            val = self.right_encoder.get()
            if val > revs0 + 1418:
                break
            self.forward(0.7)
            yield
        self.auto.after("winch", "drop%s" % tote_number)
        yield

    def auto_drive_until_liftable(self, tote_number):
        while not self.left_claw_whisker():
            self.forward(0.3)
            yield

    def drop_tote(self):
        self.winch_setpoint = self.winch_setpoint_zero
        durped = False
        while self.get_winch_revs() > self.winch_setpoint_zero + 10:
            if not durped and self.get_winch_revs() <= 15:
                self.claw_down()
                durped = True
            self.winch_set(-1.0)
            yield

    def turn_back_left(self):
        angle0 = self.gyro.getAngle()
        settle_count = 0
        while True:
            angle = self.gyro.getAngle()
            anglediff = (angle0 + 90) - angle
            if abs(anglediff) < 3:
                settle_count += 1
            else:
                settle_count = 0
            if settle_count > 20:
                break
            val = -0.08 * anglediff
            if val > 0.5:
                val = 0.5
            if val < -0.5:
                val = -0.5
            self.left_motor.set(0)
            self.right_motor.set(val)
            yield

    def turn_forward_left(self):
        angle0 = self.gyro.getAngle()
        settle_count = 0
        while True:
            angle = self.gyro.getAngle()
            anglediff = (angle0 - 90) - angle
            if abs(anglediff) < 3:
                settle_count += 1
            else:
                settle_count = 0
            if settle_count > 20:
                break
            val = -0.08 * anglediff
            if val > 0.5:
                val = 0.5
            if val < -0.5:
                val = -0.5
            self.left_motor.set(0)
            self.right_motor.set(val)
            yield

    def auto_tote_periodic(self):
        """
        for i in range(20):
            self.forward(-0.5)
            yield
        """
        for x in self.turn_back_left():
            yield
        for i in range(140):
            self.forward(0.5)
            yield
        for i in range(15):
            self.forward(-0.5)
            yield
        for x in self.turn_forward_left():
            yield
        while True:
            self.forward(0)
            yield

    # Simplest turn algorithm
    # Returns whether it is done turning
    def turn_brake(self, angle):
        print('angle: ', abs(self.gyro.getAngle()) % 360)
        if abs(self.gyro.getAngle()) % 360 < angle:
            self.pivot_clockwise(1)
        elif abs(self.gyro.getRate()) > .01:
            self.brake_rotation()
        else:
            return True
        return False

    # Turn should have a slow down so it stops at angle perfectly
    def turn(self, angle):
        slow_down_angle = 30

        remaining_angle = angle - abs(self.gyro.getAngle()) % 360

        if abs(remaining_angle) < 1 and abs(self.gyro.getRate()) < .1:
            return True
        elif abs(remaining_angle) > slow_down_angle:
            value = 1
        else:
            value = (math.sin(remaining_angle *
                              (180/slow_down_angle) - 90) + 1) / 2

        value = math.copysign(value, remaining_angle)

        self.robotdrive.tankDrive(-value, value)  # rotate
        return False

    # Autonomous mode for picking up recycling containers
    # Note: run variable "auto_mode" should be set to "container"
    # Current implementation can also pick up and score a single tote
    def auto_container_periodic(self):
        print('auto state: ', self.auto_state)
        # state "start": claw should be down to pick up totes/containers
        if self.auto_state == "start":
            self.claw_down()
            self.set_claw()
            self.auto_state = "lift"

        # state "lift": lift up to pick up container
        if self.auto_state == "lift":
            if -self.winch_encoder.get() < 500:
                self.winch_motor.set(self.winch_power.set(.5))
            else:
                self.winch_power.force(0)
                self.auto_state = "clawout"

        # state "clawout": push out solenoid
        if self.auto_state == "clawout":
            self.claw_up()
            self.set_claw()
            self.auto_state = "turn"

        # do i want to do a 180 degree turn here?
        if self.auto_state == "turn":
            done_turning = self.turn_brake(180)
            if done_turning:
                self.auto_state = "drive"

        # state "drive": drive over the bump
        if self.auto_state == "drive":
            if self.positioned_count < 190:
                self.forward(.6)
                self.positioned_count += 1
                # print('positioned_count: ', self.positioned_count)
                self.winch_motor.set(0.1 -
                                     0.01 * (-self.winch_encoder.get() - 500))
            else:
                self.positioned_count = 0
                self.auto_state = "setdown"

        # state "setdown": set container down
        if self.auto_state == "setdown":
            # print('-self.winch_encoder.get(): ', -self.winch_encoder.get())
            if -self.winch_encoder.get() > 15:
                self.winch_motor.set(-.5)
                self.brake_linear()
            else:
                self.winch_motor.set(0)
                self.auto_state = "clawin"

        # state "clawin": claw should be down to release tote/container
        if self.auto_state == "clawin":
            self.claw_down()
            self.set_claw()
            self.auto_state = "wait"

        # state "wait": waits for the claw to pull away from the tote
        if self.auto_state == "wait":
            if self.positioned_count < 20:
                self.positioned_count += 1
            else:
                self.positioned_count = 0
                self.auto_state = "backup"

        # state "backup": back up
        if self.auto_state == "backup":
            if self.positioned_count < 15:
                self.forward(-1)
                self.positioned_count += 1
            else:
                self.positioned_count = 0
                self.auto_state = "finished"

    def forward(self, val):
        self.left_motor.set(-val)
        self.right_motor.set(val)

    def pivot_clockwise(self, val):
        self.left_motor.set(-val)
        self.right_motor.set(-val)

    # Teleop Mode
    def teleopInit(self):
        self.winch_setpoint = -self.winch_encoder.get()
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
            if -self.winch_encoder.get() < 500:
                self.winch_motor.set(self.winch_power.set(.5))
            else:
                self.winch_power.force(0)
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
        self.arm_motor.set(self.arm_power.set(arm_signal))

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

    # Test Mode
    # calculates and prints values to be used in testing
    def test_mode(self):
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
        if self.left_joystick.getRawButton(6):
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
            revs = -self.winch_encoder.get()
            print('winch revolutions: ', revs)

        # Print current gyro value if left button 8 is pressed
        if self.left_joystick.getRawButton(8):
            angle = self.gyro.getAngle()
            print('angle: ', angle)

        # Reset gyro to 0 if left button 9 is pressed
        if self.left_joystick.getRawButton(9):
            self.gyro.reset()

        if self.right_joystick.getRawButton(2) or \
                self.right_joystick.getRawButton(3):
            print('winch power: ', self.winch_power.value)

        if self.left_joystick.getRawButton(2) or \
                self.left_joystick.getRawButton(3):
            print('arm power: ', self.arm_power.value)

    # Disabled Mode
    def disabledPeriodic(self):
        self.compressor.stop()

    # Helper Functions

    # Set "left" and "right" variables to the left and right
    # joystick outputs provided they are more than "joystick_threshold"
    def drive_values(self):
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

    # Moves claw into "claw_state" position
    def set_claw(self):
        self.solenoid1.set(not self.claw_state)
        self.solenoid2.set(self.claw_state)

    # Pushes claw out
    def claw_up(self):
        self.claw_state = False

    # Pulls claw in
    def claw_down(self):
        self.claw_state = True

    def left_claw_whisker(self):
        return self.left_limit_switch.get()

    def right_claw_whisker(self):
        return self.right_limit_switch.get()

    def winch_encoder_min(self):
        return self._winch_encoder_min

    def winch_encoder_max(self):
        return self._winch_encoder_min + 1162

    # Sets winch to move inputted direction
    # Maintains winch height if no winch input
    # Right joystick button 6 overrides encoder, button 7 resets encoder
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
        self.winch_motor.set(self.winch_power.set(val))

    def reset_auto(self):
        self.auto_state = "start"
        self.winch_power.force(0)

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
            if -self.winch_encoder.get() < 500:
                self.winch_motor.set(self.winch_power.set(.5))
            else:
                self.winch_power.force(0)
                self.auto_state = "clawout"

        # state "clawout": push out solenoid
        if self.auto_state == "clawout":
            self.claw_up()
            self.set_claw()
            self.auto_state = "finished"


if __name__ == "__main__":
    wpilib.run(Robot)
