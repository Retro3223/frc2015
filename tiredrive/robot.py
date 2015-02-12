import wpilib
from xbox import XboxController


def step(value, min):
    if abs(value) < min:
        value = 0
    return value


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.xbox = XboxController(self.joystick1)
        self.motor0 = wpilib.Jaguar(0)
        self.motor1 = wpilib.Jaguar(1)
        self.robotdrive = wpilib.RobotDrive(self.motor0, self.motor1)
        self.robotdrive.setInvertedMotor(
            wpilib.RobotDrive.MotorType.kRearLeft, False)
        self.robotdrive.setInvertedMotor(
            wpilib.RobotDrive.MotorType.kRearRight, False)
        self.winch_motor = wpilib.Talon(2)
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


    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.compressor.start()

    def teleopPeriodic(self):
        x = step(
            self.xbox.right_joystick_axis_v(),
            0.2,
        )
        y = step(
            self.xbox.left_joystick_axis_h(),
            -0.2,
        )
        a_x = self.accel.getX()
        self.a_x_sum += a_x
        self.a_x_count += 1
        a_y = self.accel.getY()
        self.a_y_sum += a_y
        self.a_y_count += 1
        self.robotdrive.arcadeDrive(x, y)
        # winch motor
        revs = -self.winch_encoder.get()
        winch_signal = self.xbox.right_trigger() + -self.xbox.left_trigger()
        if winch_signal > 0.1 and revs >= 1170:
            winch_signal = 0
        if winch_signal < -0.1 and revs <= 8:
            winch_signal = 0
        self.winch_motor.set(0.5 * winch_signal + .1)
        if self.xbox.A():
            print ('x sum: ', self.a_x_sum, ' x count: ', self.a_x_count)
            print ('y sum: ', self.a_y_sum, ' y count: ', self.a_y_count)
        if self.xbox.B():
            print ('revs: ', revs)

        if self.xbox.X():
            self.x_pressed_last = True
        elif self.x_pressed_last:
            self.x_pressed_last = False
            self.claw_state = not self.claw_state

        self.solenoid1.set(not self.claw_state)
        self.solenoid2.set(self.claw_state)

        if self.xbox.Y():
            print ("ultra0: ", self.ultra0.getValue())
            print ("ultra1: ", self.ultra1.getValue())

    def testPeriodic(self):
        pass
    def disabledPeriodic(self):
        self.compressor.stop()


if __name__ == "__main__":
    wpilib.run(Robot)
