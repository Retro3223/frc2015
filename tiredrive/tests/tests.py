import pytest
from mock import Mock
import math
from tiredrive.robot import Robot

def test_robot1():
    robot = Robot()
    robot.robotInit()
    robot.winch_encoder = Mock(robot.winch_encoder)
    robot.winch_encoder.get = Mock(return_value=-500)
    robot.winch_motor = Mock(robot.winch_motor)
    robot.winch_motor.set = Mock()
    robot.winch_set(-0.5)
    robot.winch_encoder.get.return_value = -10
    robot.winch_set(0.0)
    assert robot.winch_setpoint == 10
