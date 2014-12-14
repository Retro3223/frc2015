try:
    import wpilib
except ImportError:
    from pyfrc import wpilib


class Robot(wpilib.SimpleRobot):
    def __init__(self):
        pass

    def Autonomous(self):
        pass

    def Disabled(self):
        pass

    def OperatorControl(self):
        pass


if __name__ == '__main__':
    robot = Robot()
    robot.StartCompetition()
