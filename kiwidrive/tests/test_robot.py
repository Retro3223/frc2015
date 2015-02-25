from kiwidrive.robot import Robot
from .utils import assert_called_with_fuzzy
from mock import Mock


def test_arm():
    robot = Robot()
    robot.robotInit()
    arm_motor = robot.kiwidrive.arm_motor
    arm_motor.set = Mock()
    joy = robot.kiwidrive.joy
    joy.analog_arm = Mock(return_value=0)
    robot.teleopInit()
    data = [
        (0, 0.00,),
        (0, 0.00,),
        (0, 0.00,),
        (1, 0.05,),
        (1, 0.10,),
        (1, 0.15,),
        (1, 0.20,),
        (1, 0.25,),
        (1, 0.30,),
        (1, 0.30,),
        (0, 0.25,),
        (0, 0.20,),
        (0, 0.15,),
        (0, 0.10,),
        (0, 0.05,),
        (0, 0.00,),
        (0, 0.00,),
        (0, 0.00,),
        (-1, -0.05,),
        (-1, -0.10,),
        (-1, -0.15,),
        (-1, -0.20,),
        (-1, -0.25,),
        (-1, -0.30,),
        (-1, -0.30,),
    ]
    for signal, arm_motor_value in data:
        joy.analog_arm.return_value = signal
        robot.teleopPeriodic()
        assert_called_with_fuzzy(arm_motor.set, arm_motor_value)


def test_winch():
    robot = Robot()
    robot.robotInit()


def setup_winch_set_robot():
    robot = Robot()
    robot.robotInit()
    winch_encoder = robot.kiwidrive.winch_encoder
    winch_encoder.get = Mock(return_value=-500)
    winch_motor = robot.kiwidrive.winch_motor
    winch_motor.set = Mock()
    robot.teleopInit()
    return robot


def test_winch_set_safety_down():
    """
    winch_set should not drive winch below the minimum encoder value
    """
    robot = setup_winch_set_robot()
    winch_encoder = robot.kiwidrive.winch_encoder
    winch_motor = robot.kiwidrive.winch_motor
    assert robot.winch_encoder_min() == 8
    assert robot.winch_encoder_max() == 1170

    min = robot.winch_encoder_min()

    # ok
    winch_encoder.get = Mock(return_value=-(min+1))
    robot.winch_set(-1.0)
    assert_called_with_fuzzy(winch_motor.set, -0.5)

    # out
    winch_encoder.get = Mock(return_value=-min)
    robot.winch_set(-1.0)
    assert_called_with_fuzzy(winch_motor.set, 0.0)

    # out
    winch_encoder.get = Mock(return_value=-(min-1))
    robot.winch_set(-1.0)
    assert_called_with_fuzzy(winch_motor.set, 0.0)


def test_winch_set_safety_up():
    """
    winch_set should not drive winch above the maximum encoder value
    """
    robot = setup_winch_set_robot()
    winch_encoder = robot.kiwidrive.winch_encoder
    winch_motor = robot.kiwidrive.winch_motor
    assert robot.winch_encoder_min() == 8
    assert robot.winch_encoder_max() == 1170

    max = robot.winch_encoder_max()

    # ok
    winch_encoder.get = Mock(return_value=-(max-1))
    robot.winch_set(1.0)
    assert_called_with_fuzzy(winch_motor.set, 0.5)

    # out
    winch_encoder.get = Mock(return_value=-max)
    robot.winch_set(1.0)
    assert_called_with_fuzzy(winch_motor.set, 0.0)

    # out
    winch_encoder.get = Mock(return_value=-(max+1))
    robot.winch_set(1.0)
    assert_called_with_fuzzy(winch_motor.set, 0.0)
