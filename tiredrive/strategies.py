import math
from parallel_generators import ParallelGenerators


class TurnStrategy:

    def __init__(self, robot):
        self.robot = robot
        self.robot.strategies['tote'] = self

    def autonomousInit(self):
        self.auto = parallel_generators.ParallelGenerators()
        self.auto.add("back_left", self.turn_back_left())
        self.auto.add("forward1", self.forward1(), after="back_left")
        self.auto.add("brake1", self.brake1(), after="forward1")
        self.auto.add("forward_left", self.turn_forward_left(), after="brake1")
        self.auto.add("wait", self.wait(), after="forward_left")

    def auto_tote_periodic(self):
        for x in self.turn_back_left():
            yield

    def forward1(self):
        for i in range(140):
            self.robot.forward(0.5)
            yield

    def brake1(self):
        for i in range(15):
            self.robot.forward(-0.5)
            yield

    def wait(self):
        while True:
            self.robot.forward(0)
            yield

    def turn_back_left(self):
        angle0 = self.robot.gyro.getAngle()
        settle_count = 0
        while True:
            angle = self.robot.gyro.getAngle()
            anglediff = (angle0 + 90) - angle
            if abs(anglediff) < 3:
                settle_count += 1
            else:
                settle_count = 0
            if settle_count > 20:
                break
            val = -0.08 * anglediff
            if val > 0.5:
                val = 0.5
            if val < -0.5:
                val = -0.5
            self.robot.left_motor.set(0)
            self.robot.right_motor.set(val)
            yield

    def turn_forward_left(self):
        angle0 = self.robot.gyro.getAngle()
        settle_count = 0
        while True:
            angle = self.robot.gyro.getAngle()
            anglediff = (angle0 - 90) - angle
            if abs(anglediff) < 3:
                settle_count += 1
            else:
                settle_count = 0
            if settle_count > 20:
                break
            val = -0.08 * anglediff
            if val > 0.5:
                val = 0.5
            if val < -0.5:
                val = -0.5
            self.robot.left_motor.set(0)
            self.robot.right_motor.set(val)
            yield


class Auto3StraightStrategy:
    def __init__(self, robot):
        self.robot = robot
        self.robot.strategies['3-tote-straight'] = self

    def autonomousInit(self):
        auto = ParallelGenerators()
        self.robot.claw_down()
        self.winch_value = 0.0
        auto.add("claw", self.robot.maintain_claw())
        auto.add("winch", self.maintain_winch())
        auto.add("pickup1", self.auto_pickup_tote())
        auto.add("drive1", self.auto_drive_until_tote(), after="pickup1")
        auto.add("drop1", self.drop_tote(1), after="drive1")
        auto.add("drive1.5", self.auto_drive_until_liftable(), after="drop1")
        auto.add("pickup2", self.auto_pickup_tote(), after="drive1.5")
        auto.add("drive2", self.auto_drive_until_tote(), after="pickup2")
        auto.add("drop2", self.drop_tote(2), after="drive2")
        auto.add("drive2.5", self.auto_drive_until_liftable(), after="drop2")
        auto.add("pickup3", self.auto_pickup_tote(), after="drive2.5")
        self.auto = auto

    def autonomousPeriodic(self):
        self.auto.next()

    def auto_pickup_tote(self):
        robot = self.robot
        # assert robot.get_winch_revs() < 20
        tote_revs = 330
        robot.winch_setpoint = robot.winch_setpoint_zero + tote_revs
        durped = False
        while robot.get_winch_revs() < robot.winch_setpoint:
            if not durped and robot.get_winch_revs() >= 70:
                robot.claw_up()
                durped = True
            self.winch_value = 1.0
            yield
        self.winch_value = 0.0
        yield

    def auto_drive_until_tote(self):
        robot = self.robot
        val = revs0 = robot.right_encoder.get()
        while val < revs0 + 306:
            val = robot.right_encoder.get()
            robot.forward(0.7)
            yield
        yield

    def maintain_winch(self):
        while True:
            self.robot.winch_set(self.winch_value)
            yield

    def auto_drive_until_liftable(self):
        robot = self.robot
        revs0 = robot.right_encoder.get()
        while robot.right_encoder.get() <= revs0 + 70:
            robot.forward(0.7)
            yield

    def drop_tote(self, i):
        robot = self.robot
        robot.winch_setpoint = robot.winch_setpoint_zero
        durp = False
        while robot.get_winch_revs() > robot.winch_setpoint_zero + 10:
            self.winch_value = -1.0
            if not durp and robot.get_winch_revs() < robot.winch_setpoint_zero + 290 and \
                    ("drop%s" % i) in self.auto.generators:
                self.auto.add("back", self.backup())
                # put drivei.5 behind "back"
                x = self.auto.afters["drop%s" % i].pop()
                assert x[0] == ("drive%s.5" % i)
                self.auto.afters["back"] = [x]
                durp = True
            yield
        robot.claw_down()
        self.winch_value = 0.0
        yield

    def backup(self):
        for i in range(30):
            self.robot.forward(-0.7)
            yield


class ContainerStrategy:
    def __init__(self, robot, over_scoring=True):
        self.robot = robot
        if over_scoring:
            self.robot.strategies['container-overwhite'] = self
        else:
            self.robot.strategies['container-nowhite'] = self
        self.over_scoring = over_scoring

    def autonomousInit(self):
        self.auto_state = "start"
        self.positioned_count = 0

    def autonomousPeriodic(self):
        """
        Autonomous mode for picking up recycling containers
        Note: run variable "auto_mode" should be set to "container"
        Current implementation can also pick up and score a single tote
        """
        robot = self.robot
        # state "start": claw should be down to pick up totes/containers
        if self.auto_state == "start":
            robot.claw_down()
            robot.set_claw()
            self.auto_state = "lift"

        # state "lift": lift up to pick up container
        if self.auto_state == "lift":
            if robot.get_winch_revs() < 500:
                robot.winch_motor.set(.5)
            else:
                self.auto_state = "clawout"

        # state "clawout": push out solenoid
        if self.auto_state == "clawout":
            robot.claw_up()
            robot.set_claw()
            self.auto_state = "turn"

        # do i want to do a 180 degree turn here?
        if self.auto_state == "turn":
            done_turning = True
            if done_turning:
                self.auto_state = "drive"

        # state "drive": drive over the bump
        if self.auto_state == "drive":
            if self.over_scoring:
                count = 150
            else:
                count = 120
            if self.positioned_count < count:
                robot.forward(-.6)
                self.positioned_count += 1
                robot.winch_motor.set(0.1 -
                                      0.01 * (robot.get_winch_revs() - 500))
            else:
                self.positioned_count = 0
                self.auto_state = "setdown"

        # state "setdown": set container down
        if self.auto_state == "setdown":
            if robot.get_winch_revs() > 15:
                robot.winch_motor.set(-.5)
                robot.brake_linear()
            else:
                robot.winch_motor.set(0)
                self.auto_state = "clawin"

        # state "clawin": claw should be down to release tote/container
        if self.auto_state == "clawin":
            robot.claw_down()
            robot.set_claw()
            self.auto_state = "wait"

        # state "wait": waits for the claw to pull away from the tote
        if self.auto_state == "wait":
            if self.positioned_count < 20:
                self.positioned_count += 1
            else:
                self.positioned_count = 0
                self.auto_state = "backup"

        # state "backup": back up
        if self.auto_state == "backup":
            if self.positioned_count < 40:
                # turn back to the right
                self.robot.forward(-0.4)
                self.positioned_count += 1
            else:
                self.positioned_count = 0
                self.auto_state = "finished"
