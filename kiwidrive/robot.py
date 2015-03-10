import wpilib
import kiwi
import strategies as strats


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.motor1 = wpilib.Talon(0)
        self.motor2 = wpilib.Talon(1)
        self.motor3 = wpilib.Talon(2)
        self.kiwidrive = kiwi.KiwiDrive(
            self,
            self.joystick1,
            [self.motor1,
             self.motor2,
             self.motor3])

        self.pressure_switch = wpilib.DigitalInput(8)
        self.compressor = wpilib.DigitalOutput(3)
        self.solenoid1 = wpilib.Solenoid(6)
        self.solenoid2 = wpilib.Solenoid(7)

        # Initialize the arm motor
        self.arm_motor = wpilib.Talon(6)
        self.arm_power = kiwi.Smooth(0.0, 0.05)

        # Initialize the winch motor
        self.winch_motor = wpilib.Talon(8)

        # Initialize the winch encoder
        self.winch_encoder = wpilib.Encoder(0, 1)
        self._winch_encoder_min = -8

        self.last_winch_signal = 0

        self.claw_state = True
        self.claw_toggle = False

        # Initialize autonomous strategies
        self.strategies = {}
        strats.Auto3ToteStrategy(self)
        strats.ContainerStrategy(self, True)
        strats.ContainerStrategy(self, False)

        # Select which autonomous mode:
        # * "tote"
        # * "container-overwhite"
        # * "container-nowhite"
        # * "3-tote"
        self.chooser = wpilib.SendableChooser()
        for auto_mode in self.strategies.keys():
            self.chooser.addObject(auto_mode, auto_mode)
        wpilib.SmartDashboard.putData("Choice", self.chooser)
        self.auto_mode = None

    def autonomousInit(self):
        assert self.auto_mode in self.strategies.keys()
        # self.auto_mode = auto_mode
        self.winch_setpoint_zero = self.winch_setpoint = self.get_winch_revs()
        self.strategies[self.auto_mode].autonomousInit()

    def autonomousPeriodic(self):
        self.strategies[self.auto_mode].autonomousPeriodic()

    def teleopInit(self):
        self.winch_setpoint = self.get_winch_revs()
        #self.kiwidrive.Enable()

    def disabledInit(self):
        self.kiwidrive.Disable()

    def disabledPeriodic(self):
        self.compressor.set(0)

    def teleopPeriodic(self):
        self.kiwidrive.Drive_Tank()
        self.set_compressor()
        #self.drive_arm()
        self.drive_claw()
        self.drive_winch()

        # If the right joystick slider is down, also run test mode
        if self.kiwidrive.joy.digital_test():
            self.test_mode()

    def set_compressor(self):
        psv = self.pressure_switch.get()
        if psv == 1:
            self.compressor.set(0)
        elif psv == 0:
            self.compressor.set(1)
        else:
            print("badurk?")

    def testPeriodic(self):
        pass

    def drive_arm(self):
        # Feed arm controller raw values from the joystick
        # Left joystick button 3 goes forward, 2 goes backward
        arm_signal = self.kiwidrive.joy.analog_arm()
        self.arm_motor.set(self.arm_power.set(arm_signal * .3))

    def drive_claw(self):
        # Handle piston in and out
        # Right joystick trigger button toggles claw in or out
        if self.kiwidrive.joy.digital_claw():
            self.claw_toggle = True
        elif self.claw_toggle:
            self.claw_toggle = False
            self.claw_state = not self.claw_state
            self.set_claw()

    def drive_winch(self):
        # Feed winch controller raw values from the joystick
        winch_signal = self.kiwidrive.joy.analog_winch()
        # Right joystick button 6 overrides encoder,
        # button 7 resets encoder
        self.winch_set(winch_signal)

    def maintain_claw(self):
        while True:
            self.set_claw()
            yield

    def get_winch_revs(self):
        return self.winch_encoder.get()

    def winch_encoder_min(self):
        return self._winch_encoder_min

    def winch_encoder_max(self):
        return self._winch_encoder_min + 1720

    def set_claw(self):
        """
        # Moves claw into "claw_state" position
        """
        self.solenoid1.set(self.claw_state)
        self.solenoid2.set(self.claw_state)

    def test_mode(self):
        """
        # Test Mode
        # calculates and prints values to be used in testing
        """
        if self.kiwidrive.joy.show_limit_switches():
            print("left limit switch: ", self.left_claw_whisker())
            print("right limit switch: ", self.right_claw_whisker())

        if self.kiwidrive.joy.show_winch_encoder():
            print('winch revolutions: ', self.get_winch_revs())

        if self.kiwidrive.joy.show_gyro():
            angle = self.gyro.getAngle()
            print('angle: ', angle)

        if self.kiwidrive.joy.show_arm():
            print('arm power: ', self.arm_power.value)

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
        if self.kiwidrive.joy.digital_winch_encoder_reset():
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
            if not (self.kiwidrive.joy.digital_winch_override()):
                # Stop the winch if it is going out of bounds
                if (((signal > 0.1 and revs >= self.winch_encoder_max()) or
                     (signal < -0.1 and revs <= self.winch_encoder_min()))):
                    signal = 0
            val = signal

        # Sets the winch motor's value
        self.winch_motor.set(val)

    def maintain_winch(self):
        while True:
            self.winch_set(0)
            yield

if __name__ == "__main__":
    wpilib.run(Robot)
