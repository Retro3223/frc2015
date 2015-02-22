import wpilib
import kiwi


class Robot(wpilib.IterativeRobot):
    def robotInit(self):
        self.joystick1 = wpilib.Joystick(0)
        self.motor1 = wpilib.Talon(0)
        self.motor2 = wpilib.Talon(1)
        self.motor3 = wpilib.Talon(2)
        self.kiwidrive = kiwi.KiwiDrive(
            self.joystick1,
            [self.motor1,
             self.motor2,
             self.motor3])

        # Select which autonomous mode: "tote", "container", "tripletote"
        self.auto_mode = "3-tote-straight"

    @property
    def strategies(self):
        return self.kiwidrive.strategies

    def autonomousInit(self):
        self.kiwidrive.autonomousInit(self.auto_mode)

    def autonomousPeriodic(self):
        self.kiwidrive.autonomousPeriodic()

    def teleopInit(self):
        self.kiwidrive.Enable()

    def disabledInit(self):
        self.kiwidrive.Disable()

    def teleopPeriodic(self):
        self.kiwidrive.Drive()

    def testPeriodic(self):
        pass

    def winch_encoder_min(self):
        return self.kiwidrive.winch_encoder_min()

    def winch_encoder_max(self):
        return self.kiwidrive.winch_encoder_max()

    def winch_set(self, signal):
        return self.kiwidrive.winch_set(signal)


if __name__ == "__main__":
    wpilib.run(Robot)
