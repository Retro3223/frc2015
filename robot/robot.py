import wpilib
from kiwidrive import KiwiDrive
from xbox import XboxController


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.xbox = XboxController(self.joystick1)
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
        #self.kiwidrive.Drive()

        if self.xbox.right_trigger():
            self.motor1.set(0.1)
            self.motor2.set(0.1)
            self.motor3.set(0.1)
        elif self.xbox.left_trigger():
            self.motor1.set(-0.1)
            self.motor2.set(-0.1)
            self.motor3.set(-0.1)
        else:
            self.motor1.set(0)
            self.motor2.set(0)
            self.motor3.set(0)
        #print (self.kiwidrive.gyro.getRate())
    def testPeriodic(self):
        pass



if __name__ == "__main__":
    wpilib.run(Robot)
