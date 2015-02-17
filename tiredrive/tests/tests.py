from mock import Mock
from tiredrive.robot import Robot, ParallelGenerators


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


def test_parallel_generators():
    pg = ParallelGenerators()
    pg.add("1", range(4))
    assert {"1": 0} == pg.next()
    assert {"1": 1} == pg.next()
    pg.add("2", range(3))
    assert {"1": 2, "2": 0} == pg.next()
    assert {"1": 3, "2": 1} == pg.next()
    assert {"2": 2} == pg.next()


def test_sequential_generators():
    pg = ParallelGenerators()
    pg.add("1", range(4))
    pg.add("2", range(3))
    pg.after("2", "1")
    assert {"1": 0} == pg.next()
    assert {"1": 1} == pg.next()
    assert {"1": 2} == pg.next()
    assert {"1": 3} == pg.next()
    assert {} == pg.next()
    assert {"2": 0} == pg.next()
    assert {"2": 1} == pg.next()
    assert {"2": 2} == pg.next()


def test_3_totes():
    robot = Robot()
    robot.robotInit()
    robot.winch_encoder = Mock(robot.winch_encoder)
    robot.winch_encoder.get = Mock(return_value=-8)
    assert 8 == robot.get_winch_revs()
    robot.winch_motor = Mock(robot.winch_motor)
    robot.winch_motor.set = Mock()
    robot.forward = Mock()
    robot.right_ultrasonic_sensor.getValue = Mock(return_value=300)
    robot.auto_mode = "3-tote-straight"
    robot.autonomousInit()

    assert "claw" in robot.auto.generators
    assert "winch" not in robot.auto.generators
    assert "pickup1" in robot.auto.generators

    robot.autonomousPeriodic()
    assert robot.winch_setpoint == 328
    assert "pickup1" in robot.auto.generators
    robot.winch_motor.set.assert_called_with(0.5)

    robot.winch_encoder.get.return_value = -299
    robot.autonomousPeriodic()

    assert "pickup1" in robot.auto.generators
    robot.winch_motor.set.assert_called_with(0.5)
    robot.winch_encoder.get.return_value = -390
    robot.autonomousPeriodic()

    robot.winch_motor.set.assert_called_with(0.5)
    assert "pickup1" not in robot.auto.generators
    assert "drive1" in robot.auto.generators

    robot.autonomousPeriodic()
    robot.winch_motor.set.assert_called_with(0.1)
    assert "drive1" in robot.auto.generators

    assert "claw" in robot.auto.generators
    assert "winch" in robot.auto.generators
    assert "pickup1" not in robot.auto.generators
    robot.autonomousPeriodic()
    assert "drive1" in robot.auto.generators
    robot.winch_motor.set.assert_called_with(0.1)
    robot.forward.assert_called_with(0.7)
    robot.right_ultrasonic_sensor.getValue.return_value = 140
    robot.autonomousPeriodic()
    robot.autonomousPeriodic()
    """
    assert "drive1" not in robot.auto.generators
    assert "drop1" in robot.auto.generators

    assert "claw" in robot.auto.generators
    assert "winch" not in robot.auto.generators
    assert "pickup1" not in robot.auto.generators
    assert "drive1" not in robot.auto.generators

    robot.winch_encoder.get.return_value = -199
    robot.autonomousPeriodic()
    robot.winch_motor.set.assert_called_with(-0.5)
    robot.winch_encoder.get.return_value = -10
    robot.autonomousPeriodic()
    assert "drop1" not in robot.auto.generators
    robot.autonomousPeriodic()
    robot.winch_motor.set.assert_called_with(0.1)
    """
