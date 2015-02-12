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

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        x = step(
            self.xbox.right_joystick_axis_v(),
            0.2,
        )
        y = step(
            self.xbox.left_joystick_axis_h(),
            -0.2,
        )
        self.robotdrive.arcadeDrive(x, y)
        # winch motor
        self.winch_motor.set(0.2 * (self.xbox.right_trigger()+self.xbox.left_trigger())
        

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
