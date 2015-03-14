from parallel_generators import ParallelGenerators


class TurnStrategy:
    def __init__(self, robot):
        self.robot = robot
        self.robot.strategies['turn'] = self

    def autonomousInit(self):
        self.braking = False
        self.auto = ParallelGenerators()
        self.auto.add("turn", self.turn(90))
        self.auto.add("wait", self.wait(), after="turn")
        self.auto.add("brake", self.brake())

    def autonomousPeriodic(self):
        self.auto.next()

    def turn(self, angle):
        angle0 = self.robot.gyro.getAngle()
        while abs(self.robot.gyro.getAngle() - angle0) % 360 < (angle-30):
            self.robot.pivot_clockwise(1)
            yield
        for i in range(20):
            self.braking = True
            yield
        self.braking = False
        yield

    def brake(self):
        while True:
            if self.braking:
                self.robot.brake_on()
            else:
                self.robot.brake_off()
            yield

    def wait(self):
        while True:
            self.robot.forward(0)
            yield


class Auto3ToteStrategy:
    def __init__(self, robot):
        self.robot = robot
        self.robot.strategies['3-tote'] = self

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
        claw_raised = False
        while robot.get_winch_revs() < robot.winch_setpoint:
            if not claw_raised and robot.get_winch_revs() >= 70:
                robot.claw_up()
                claw_raised = True
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

    def maintain_winch(self):
        while True:
            self.robot.winch_set(self.winch_value)
            yield

    def both_whiskers(self):
        robot = self.robot
        return (robot.right_claw_whisker() and
                robot.left_claw_whisker())

    def auto_drive_until_liftable(self):
        robot = self.robot
        while not self.both_whiskers():
            robot.forward(0.7)
            yield

    def drop_tote(self, i):
        robot = self.robot
        robot.winch_setpoint = robot.winch_setpoint_zero
        back_scheduled = False
        trigger_revs = robot.winch_setpoint_zero + 290

        def schedule_backup_maybe():
            nonlocal back_scheduled
            if not back_scheduled and \
                    robot.get_winch_revs() < trigger_revs and \
                    ("drop%s" % i) in self.auto.generators:
                self.auto.add("backup", self.backup())
                # schedule drivei.5 behind "backup"
                x = self.auto.afters["drop%s" % i].pop()
                assert x[0] == ("drive%s.5" % i)
                self.auto.afters["backup"] = [x]
                back_scheduled = True

        while robot.get_winch_revs() > robot.winch_setpoint_zero + 10:
            self.winch_value = -1.0
            schedule_backup_maybe()
            yield
        robot.claw_down()
        self.winch_value = 0.0
        yield

    def backup(self):
        for i in range(35):
            self.robot.forward(-0.7)
            yield


class ContainerStrategy:
    def __init__(self, robot, over_scoring=True, drop=True):
        self.robot = robot
        strategy_name = 'container'
        self.over_scoring = over_scoring
        if over_scoring:
            strategy_name += '-overwhite'
        else:
            strategy_name += '-nowhite'
        self.drop = drop
        if drop:
            strategy_name += '-drop'
        else:
            strategy_name += '-nodrop'
        self.robot.strategies[strategy_name] = self

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
                if self.drop:
                    self.auto_state = "setdown"
                else:
                    self.auto_state = 'finished'

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
