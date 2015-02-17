from mock import Mock
from .utils import assert_called_with_fuzzy
from tiredrive.robot import Robot


def setup_winch_set_robot():
    robot = Robot()
    robot.robotInit()
    robot.winch_encoder = Mock(robot.winch_encoder)
    robot.winch_encoder.get = Mock(return_value=-500)
    robot.winch_motor = Mock(robot.winch_motor)
    robot.winch_motor.set = Mock()
    robot.teleopInit()
    return robot


def test_winch_set_smooths():
    """
    winch_set should smooth up to desired value
    """
    robot = setup_winch_set_robot()
    data = [(-1.0, -0.1),
            (-1.0, -0.2),
            (-1.0, -0.3),
            (-1.0, -0.4),
            (-1.0, -0.5),
            (-1.0, -0.5),
            (0.0, -0.4),
            (0.0, -0.3),
            (0.0, -0.2),
            (0.0, -0.1),
            (0.0,  0.0),
            ]
    for signal, expected in data:
        robot.winch_set(signal)
        assert_called_with_fuzzy(robot.winch_motor.set, expected)


def test_winch_set_safety_down():
    """
    winch_set should not drive winch below the minimum encoder value
    """
    robot = setup_winch_set_robot()
    assert robot.winch_encoder_min() == 8
    assert robot.winch_encoder_max() == 1170

    min = robot.winch_encoder_min()

    # ok
    robot.winch_encoder.get = Mock(return_value=-(min+1))
    robot.winch_set(-1.0)
    assert_called_with_fuzzy(robot.winch_motor.set, -0.1)

    # out
    robot.winch_encoder.get = Mock(return_value=-min)
    robot.winch_set(-1.0)
    assert_called_with_fuzzy(robot.winch_motor.set, 0.0)

    # out
    robot.winch_encoder.get = Mock(return_value=-(min-1))
    robot.winch_set(-1.0)
    assert_called_with_fuzzy(robot.winch_motor.set, 0.0)


def test_winch_set_safety_up():
    """
    winch_set should not drive winch above the maximum encoder value
    """
    robot = setup_winch_set_robot()
    assert robot.winch_encoder_min() == 8
    assert robot.winch_encoder_max() == 1170

    max = robot.winch_encoder_max()

    # ok
    robot.winch_encoder.get = Mock(return_value=-(max-1))
    robot.winch_set(1.0)
    assert_called_with_fuzzy(robot.winch_motor.set, 0.1)

    # out
    robot.winch_encoder.get = Mock(return_value=-max)
    robot.winch_set(1.0)
    assert_called_with_fuzzy(robot.winch_motor.set, 0.0)

    # out
    robot.winch_encoder.get = Mock(return_value=-(max+1))
    robot.winch_set(1.0)
    assert_called_with_fuzzy(robot.winch_motor.set, 0.0)


def test_get_winch_revs():
    """
    get_winch_revs should return the revolutions of the winch encoder
    """
    robot = setup_winch_set_robot()
    robot.winch_encoder.get = Mock(return_value=-8)
    assert 8 == robot.get_winch_revs()


def test_3_totes():
    """
    auto_mode='3-tote-straight' should:
        pick up prepositioned tote A,
        then drive forward until ~1 foot before next tote (B)
        then drop tote A on tote B
        then drive forward until positioned to pick up tote B
    """
    robot = Robot()
    robot.robotInit()
    robot.winch_encoder = Mock(robot.winch_encoder)
    robot.winch_encoder.get = Mock(return_value=-8)
    robot.winch_set = Mock()
    robot.forward = Mock()
    robot.right_ultrasonic_sensor.getValue = Mock(return_value=300)
    robot.auto_mode = "3-tote-straight"
    robot.autonomousInit()

    auto = robot.strategies[robot.auto_mode].auto

    assert "claw" in auto.generators
    assert "winch" not in auto.generators
    assert "pickup1" in auto.generators

    robot.autonomousPeriodic()
    assert robot.winch_setpoint == 328
    assert "pickup1" in auto.generators
    robot.winch_set.assert_called_with(1.0)
    robot.autonomousPeriodic()

    robot.winch_encoder.get.return_value = -299
    robot.autonomousPeriodic()

    assert "pickup1" in auto.generators
    robot.winch_set.assert_called_with(1.0)
    robot.winch_encoder.get.return_value = -390
    robot.autonomousPeriodic()

    robot.winch_set.assert_called_with(1.0)
    assert "pickup1" not in auto.generators
    assert "drive1" in auto.generators

    robot.autonomousPeriodic()
    robot.winch_set.assert_called_with(0)
    assert "drive1" in auto.generators

    assert "claw" in auto.generators
    assert "winch" in auto.generators
    assert "pickup1" not in auto.generators
    robot.autonomousPeriodic()
    assert "drive1" in auto.generators
    robot.winch_set.assert_called_with(0)
    robot.forward.assert_called_with(0.7)
    robot.right_ultrasonic_sensor.getValue.return_value = 140
    robot.autonomousPeriodic()
    robot.autonomousPeriodic()
    """
    assert "drive1" not in auto.generators
    assert "drop1" in auto.generators

    assert "claw" in auto.generators
    assert "winch" not in auto.generators
    assert "pickup1" not in auto.generators
    assert "drive1" not in auto.generators

    robot.winch_encoder.get.return_value = -199
    robot.autonomousPeriodic()
    robot.winch_motor.set.assert_called_with(-0.5)
    robot.winch_encoder.get.return_value = -10
    robot.autonomousPeriodic()
    assert "drop1" not in auto.generators
    robot.autonomousPeriodic()
    robot.winch_motor.set.assert_called_with(0.1)
    """
