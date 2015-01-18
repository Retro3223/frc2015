import wpilib
from xbox import XboxController


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.xbox = XboxController(self.joystick1)
        self.motor0 = wpilib.Jaguar(0)
        #self.motor2 = wpilib.Jaguar(1)

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        if self.xbox.A():
            self.motor0.set(0.5)
        else:
            self.motor0.set(0)

    def testPeriodic(self):
        pass



if __name__ == "__main__":
    wpilib.run(Robot)
