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
    def teleopInit(self):
        self.kiwidrive.Enable()
    def disabledInit(self):
        self.kiwidrive.Disable()
    def teleopPeriodic(self):
        self.kiwidrive.Drive()

        #print (self.kiwidrive.gyro.getAngle())
    def testPeriodic(self):
        pass



if __name__ == "__main__":
    wpilib.run(Robot)
