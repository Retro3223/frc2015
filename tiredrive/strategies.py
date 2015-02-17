from parallel_generators import ParallelGenerators


class TurnStrategy:

    def __init__(self, robot):
        self.robot = robot
        self.robot.strategies['tote'] = self

    def autonomousInit(self):
        self.auto = ParallelGenerators()
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
        auto.add("claw", self.robot.maintain_claw())
        auto.add("pickup1", self.auto_pickup_tote(1))
        auto.add("winch", self.robot.maintain_winch(), after="pickup1")
        auto.add("drive1", self.auto_drive_until_tote(1), after="pickup1")
        auto.add("drop1", self.drop_tote(), after="drive1")
        auto.add("drive1.5", self.auto_drive_until_liftable(1), after="drop1")
        self.auto = auto

    def autonomousPeriodic(self):
        self.auto.next()

    def auto_pickup_tote(self, tote_count):
        robot = self.robot
        tote_revs = 320
        assert robot.get_winch_revs() < 20
        robot.winch_setpoint = robot.winch_setpoint_zero + \
            tote_revs * tote_count
        durped = False
        while robot.get_winch_revs() < robot.winch_setpoint:
            if not durped and robot.get_winch_revs() >= 70:
                robot.claw_up()
                durped = True
            robot.winch_set(1.0)
            yield

    def auto_drive_until_tote(self, tote_number):
        robot = self.robot
        revs0 = robot.right_encoder.get()
        while True:
            val = robot.right_encoder.get()
            if val > revs0 + 1418:
                break
            robot.forward(0.7)
            yield
        self.auto.after("winch", "drop%s" % tote_number)
        yield

    def auto_drive_until_liftable(self, tote_number):
        robot = self.robot
        while not robot.left_claw_whisker():
            robot.forward(0.3)
            yield

    def drop_tote(self):
        robot = self.robot
        robot.winch_setpoint = robot.winch_setpoint_zero
        durped = False
        while robot.get_winch_revs() > robot.winch_setpoint_zero + 10:
            if not durped and robot.get_winch_revs() <= 15:
                robot.claw_down()
                durped = True
            robot.winch_set(-1.0)
            yield


class ContainerStrategy:
    def __init__(self, robot):
        self.robot = robot
        self.robot.strategies['container'] = self

    def autonomousInit(self):
        self.auto_state = "start"
        self.positioned_count = 0

    def autonomousPeriodic(self):
        """
        Autonomous mode for picking up recycling containers
        Note: run variable "auto_mode" should be set to "container"
        Current implementation can also pick up and score a single tote
        """
        print('auto state: ', self.auto_state)
        robot = self.robot
        # state "start": claw should be down to pick up totes/containers
        if self.auto_state == "start":
            robot.claw_down()
            robot.set_claw()
            self.auto_state = "lift"

        # state "lift": lift up to pick up container
        if self.auto_state == "lift":
            if robot.get_winch_revs() < 500:
                robot.winch_motor.set(robot.winch_power.set(.5))
            else:
                robot.winch_power.force(0)
                self.auto_state = "clawout"

        # state "clawout": push out solenoid
        if self.auto_state == "clawout":
            robot.claw_up()
            robot.set_claw()
            self.auto_state = "turn"

        # do i want to do a 180 degree turn here?
        if self.auto_state == "turn":
            done_turning = self.turn_brake(180)
            if done_turning:
                self.auto_state = "drive"

        # state "drive": drive over the bump
        if self.auto_state == "drive":
            if self.positioned_count < 190:
                robot.forward(.6)
                self.positioned_count += 1
                # print('positioned_count: ', self.positioned_count)
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
            if self.positioned_count < 15:
                robot.forward(-1)
                self.positioned_count += 1
            else:
                self.positioned_count = 0
                self.auto_state = "finished"

    # Simplest turn algorithm
    # Returns whether it is done turning
    def turn_brake(self, angle):
        print('angle: ', abs(self.robot.gyro.getAngle()) % 360)
        if abs(self.robot.gyro.getAngle()) % 360 < angle:
            self.robot.pivot_clockwise(1)
        elif abs(self.robot.gyro.getRate()) > .01:
            self.robot.brake_rotation()
        else:
            return True
        return False
