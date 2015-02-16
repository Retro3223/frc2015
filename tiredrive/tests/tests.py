from mock import Mock
from tiredrive.robot import Robot


def test_robot1():
    robot = Robot()
    robot.robotInit()
    robot.winch_encoder = Mock(robot.winch_encoder)
    robot.winch_encoder.get = Mock(return_value=-500)
    robot.winch_motor = Mock(robot.winch_motor)
    robot.winch_motor.set = Mock()
    robot.winch_set(-0.5)
    robot.winch_motor.set.assert_called_with(-0.25)
    robot.winch_set(0.0)
    robot.winch_motor.set.assert_called_with(0.1)
    assert robot.winch_setpoint == 500
    robot.winch_encoder.get = Mock(return_value=-1200)
    robot.winch_set(-0.5)


def test_robot2():
    robot = Robot()
    robot.robotInit()
    robot.winch_encoder = Mock(robot.winch_encoder)
    robot.winch_encoder.get = Mock(return_value=-500)
    robot.winch_motor = Mock(robot.winch_motor)
    robot.winch_motor.set = Mock()
    robot.winch_set(0.5)
    robot.winch_motor.set.assert_called_with(0.25)
    robot.winch_encoder.get = Mock(return_value=-1200)
    robot.winch_set(0.5)
    robot.winch_motor.set.assert_called_with(0)
    robot.winch_set(0.0)
    robot.winch_motor.set.assert_called_with(0.1)
    assert robot.winch_setpoint == 1200
