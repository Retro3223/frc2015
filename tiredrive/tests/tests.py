import pytest
from mock import Mock
import math
from tiredrive.robot import Robot, Smooth

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
    
def test_smooth():
    smoothie  = Smooth(0, 0.1)
    for i in range(5):
        assert .1 * (i+1) == smoothie.set(0.5)
