import wpilib
from kiwidrive import KiwiDrive


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.motor1 = wpilib.Talon(0)
        self.motor2 = wpilib.Talon(1)
        self.motor3 = wpilib.Talon(2)
        self.kiwidrive = KiwiDrive(
            self.joystick1,
            [self.motor1,
             self.motor2,
             self.motor3])

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        self.kiwidrive.Drive()

    def testPeriodic(self):
        pass



if __name__ == "__main__":
    wpilib.run(Robot)
