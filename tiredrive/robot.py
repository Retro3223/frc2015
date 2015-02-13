import wpilib


def step(value, min):
    if abs(value) < min:
        value = 0
    return value

def step_range(value, min, max, default):
    if not (min <= value <= max):
        value = default
    return value


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)
        self.motor0 = wpilib.Jaguar(0)
        self.motor1 = wpilib.Jaguar(1)
        self.robotdrive = wpilib.RobotDrive(self.motor0, self.motor1)
        self.robotdrive.setInvertedMotor(
            wpilib.RobotDrive.MotorType.kRearLeft, False)
        self.robotdrive.setInvertedMotor(
            wpilib.RobotDrive.MotorType.kRearRight, False)
        self.winch_motor = wpilib.Talon(2)
        self.arm_motor = wpilib.Talon(3)
        self.accel = wpilib.BuiltInAccelerometer()
        self.a_x_sum = 0.0
        self.a_x_count = 0
        self.a_y_sum = 0.0
        self.a_y_count = 0
        self.winch_encoder = wpilib.Encoder(1,2)
        #self.winch_encoder.setReverseDirection(True)
        self.compressor = wpilib.Compressor(0)
        self.solenoid1 = wpilib.Solenoid(1)
        self.solenoid2 = wpilib.Solenoid(2)
        self.claw_state = True
        self.x_pressed_last = False
        self.ultra0 = wpilib.AnalogInput(0)
        self.ultra1 = wpilib.AnalogInput(1)
        self.optical1 = wpilib.DigitalInput(3)
        self.auto_mode = "tote"
        self.dog = wpilib.MotorSafety()
        self.dog.setSafetyEnabled(False)
        self.dog.setExpiration(1.75)


    def autonomousInit(self):
        self.auto_state = "start"
        self.positioned_count = 0

    def autonomousPeriodic(self):
        self.dog.feed()
        if self.auto_mode == "container":
            self.autoContainerPeriodic()
        elif self.auto_mode == "tote":
            self.autoTotePeriodic()

    def autoContainerPeriodic(self):
        pass

    def autoTotePeriodic(self):
        if self.auto_state == "start":
            right_dist = step_range(self.ultra0.getValue(), 50, 200, 200)
            left_dist = step_range(self.ultra1.getValue(), 50, 200, 200)
            if right_dist < 70 and left_dist < 70:
                self.auto_state = "positioned"
            elif right_dist >= 70 and left_dist < 70:
                val1 = (0.5 * abs(right_dist - 70) / 200.0)
                if abs(val1) < 0.2:
                    val1 = 0.2
                val0 = 0.0
                print (val0, val1)
                self.motor0.set(val0)
                self.motor1.set(val1)
            elif right_dist < 70 and left_dist >= 70:
                val0 = 0.0
                val1 = (0.5 * abs(left_dist - 70) / 200.0)
                if abs(val1) < 0.2:
                    val1 = 0.2
                self.motor0.set(val0)
                self.motor1.set(val1)
            else:
                val0 = (-0.5 * abs(right_dist - 70) / 200.0)
                if abs(val0) < 0.2:
                    val0 = -0.2
                val1 = (0.5 * abs(left_dist - 70) / 200.0)
                if abs(val1) < 0.2:
                    val1 = 0.2
                self.motor0.set(val0)
                self.motor1.set(val1)
        elif self.auto_mode == "positioned":
            self.positioned_count += 1
            self.winch_set(0.5);
            if self.positioned_count > 40:
                self.claw_up()

    def set_claw(self):
        self.solenoid1.set(not self.claw_state)
        self.solenoid2.set(self.claw_state)

    def claw_up(self):
        self.claw_state = False

    def claw_down(self):
        self.claw_state = True

    def winch_set(self, winch_signal):
        """
        winch_signal=0 -> maintain winch position
        winch_signal>0 -> winch up?
        winch_signal<0 -> winch down?
        """
        revs = -self.winch_encoder.get()
        if not (self.joystick2.getRawButton(4)):
            if winch_signal > 0.1 and revs >= 1170:
                winch_signal = 0
            if winch_signal < -0.1 and revs <= 8:
                winch_signal = 0
        val = 0.5 * winch_signal + .1
        self.winch_motor.set(val)

    def teleopInit(self):
        self.compressor.start()

    def teleopPeriodic(self):
        x = step(
            self.joystick1.getRawAxis(1),
            0.2,
        )
        y = step(
            self.joystick2.getRawAxis(1),
            -0.2,
        )
        a_x = self.accel.getX()
        self.a_x_sum += a_x
        self.a_x_count += 1
        a_y = self.accel.getY()
        self.a_y_sum += a_y
        self.a_y_count += 1
        self.robotdrive.tankDrive(x, y)
        # winch motor
        winch_signal = self.joystick2.getRawButton(3) + -self.joystick2.getRawButton(2)
        self.winch_set(winch_signal)
        arm_signal = self.joystick1.getRawButton(3) + -self.joystick1.getRawButton(2)
        self.arm_motor.set(0.3 * arm_signal)
        if self.joystick1.getRawButton(8):
            print ('x sum: ', self.a_x_sum, ' x count: ', self.a_x_count)
            print ('y sum: ', self.a_y_sum, ' y count: ', self.a_y_count)
        if self.joystick2.getRawButton(5):
            self.winch_encoder.reset()
        if self.joystick1.getRawButton(9):
            revs = -self.winch_encoder.get()
            print ('revs: ', revs)

        if self.joystick2.getRawButton(1):
            self.x_pressed_last = True
        elif self.x_pressed_last:
            self.x_pressed_last = False
            self.claw_state = not self.claw_state

        self.set_claw()

        if self.joystick1.getRawButton(10):
            print ("ultra0: ", self.ultra0.getValue())
            print ("ultra1: ", self.ultra1.getValue())
            print ("optical1: ", self.optical1.get())

    def testPeriodic(self):
        pass

    def disabledPeriodic(self):
        self.compressor.stop()


if __name__ == "__main__":
    wpilib.run(Robot)
