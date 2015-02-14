import wpilib

def step(value, min):
    if abs(value) < min:
        value = 0
    return value

def step_range(value, min, max, default):
    if not (min <= value <= max):
        value = default
    return value


#Robot object definition
class Robot(wpilib.IterativeRobot):
    #Initialize all of the sensors and controllers on the robot
    def robotInit(self):
        #Initialize the Joysticks
        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)

        #Initialize the drive motors
        self.left_motor = wpilib.Talon(0)
        self.right_motor = wpilib.Talon(1)

        #Initialize the drive system
        self.robotdrive = wpilib.RobotDrive(self.left_motor, self.right_motor)

        #Invert the motors so that they drive the right way
        self.robotdrive.setInvertedMotor(
            wpilib.RobotDrive.MotorType.kRearLeft, False)
        self.robotdrive.setInvertedMotor(
            wpilib.RobotDrive.MotorType.kRearRight, False)

        #Initialize the winch motor
        self.winch_motor = wpilib.Talon(2)

        #Initialize the arm motor
        self.arm_motor = wpilib.Talon(3)

        #Initialize the accelerometer
        self.accel = wpilib.BuiltInAccelerometer()
        self.a_x_sum = 0.0
        self.a_x_count = 0
        self.a_y_sum = 0.0
        self.a_y_count = 0

        self.last_winch_signal = 0

        #Initialize the gyro
        self.gyro = wpilib.Gyro(0)

        #Initialize the winch encoder
        self.winch_encoder = wpilib.Encoder(1,2)
        self.winch_setpoint = -self.winch_encoder.get()

        #Initialize the compressor
        self.compressor = wpilib.Compressor(0)

        #Initialize the pneumatic solenoids
        self.solenoid1 = wpilib.Solenoid(1)
        self.solenoid2 = wpilib.Solenoid(2)

        #Initialize the ultrasonic sensors
        self.left_ultrasonic_sensor = wpilib.AnalogInput(1)
        self.right_ultrasonic_sensor = wpilib.AnalogInput(2)

        #Initialize the optical sensors
        self.left_optical_sensor = wpilib.DigitalInput(3)
        self.right_optical_sensor = wpilib.DigitalInput(4)

        #Initialize the limit switches
        print ("ofoof")
        self.left_limit_switch = wpilib.DigitalInput(6)
        self.right_limit_switch = wpilib.DigitalInput(5)

        #Initialize the compressor watchdog
        self.dog = wpilib.MotorSafety()
        self.dog.setSafetyEnabled(False)
        self.dog.setExpiration(1.75)

        #Set run variables
        self.auto_mode = "tote"
        self.claw_state = True
        self.x_pressed_last = False

    #Autonomous Mode
    def autonomousInit(self):
        self.auto_state = "start"
        self.positioned_count = 0

    def autonomousPeriodic(self):
        self.dog.feed()
        if self.auto_mode == "container":
            self.autoContainerPeriodic()
        elif self.auto_mode == "tote":
            self.autoTotePeriodic()

    #Autonomous mode for picking up recycling containers
    #Note: run variable "auto_mode" should be set to "container"
    def autoContainerPeriodic(self):
        pass

    def autoTotePeriodic2(self):
        while not (self.left_claw_whisker() and self.right_claw_whisker()):
            if not self.left_claw_whisker() and not self.right_claw_whisker():
                pass
            yield
    #Autonomous mode for picking up totes
    #Note: run variable "auto_mode" should be set to "tote"
    def autoTotePeriodic(self):
        if self.auto_state == "start":
            right_dist = step_range(self.left_ultrasonic_sensor.getValue(), 50, 200, 200)
            left_dist = step_range(self.right_ultrasonic_sensor.getValue(), 50, 200, 200)
            if right_dist < 70 and left_dist < 70:
                self.auto_state = "positioned"
            elif right_dist >= 70 and left_dist < 70:
                val1 = (0.5 * abs(right_dist - 70) / 200.0)
                if abs(val1) < 0.2:
                    val1 = 0.2
                val0 = 0.0
                print (val0, val1)
                self.left_motor.set(val0)
                self.right_motor.set(val1)
            elif right_dist < 70 and left_dist >= 70:
                val0 = 0.0
                val1 = (0.5 * abs(left_dist - 70) / 200.0)
                if abs(val1) < 0.2:
                    val1 = 0.2
                self.left_motor.set(val0)
                self.right_motor.set(val1)
            else:
                val0 = (-0.5 * abs(right_dist - 70) / 200.0)
                if abs(val0) < 0.2:
                    val0 = -0.2
                val1 = (0.5 * abs(left_dist - 70) / 200.0)
                if abs(val1) < 0.2:
                    val1 = 0.2
                self.left_motor.set(val0)
                self.right_motor.set(val1)
        elif self.auto_mode == "positioned":
            self.positioned_count += 1
            self.winch_set(0.5);
            if self.positioned_count > 40:
                self.claw_up()

    #Teleop Mode
    def teleopInit(self):
        self.compressor.start()

    def teleopPeriodic(self):
        #Set "left" and "right" variables to the left and right
        #joystick outputs provided they are more than "threshold"
        joystick_threshold = 0.2
        left = step(
            self.left_joystick.getRawAxis(1),
            joystick_threshold,
        )
        right = step(
            self.right_joystick.getRawAxis(1),
            -joystick_threshold,
        )

        #Fuzzy match where if the left and right joysticks are moved about the same,
        #    then it moves the tankDrive the average of the two values
        if abs(left - right) < .1 :
            left = right = (left + right) / 2.0

        #Feed joystick values into drive system
        self.robotdrive.tankDrive(left, right)

        #Feed winch controller raw values from the joystick
        #Right joystick button 3 raises winch, button 2 lowers winch
        winch_signal = self.right_joystick.getRawButton(3) + -self.right_joystick.getRawButton(2)
        self.winch_set(winch_signal)

        #Feed arm controller raw values from the joystick
        #Left joystick button 3 goes forward, 2 goes backward
        arm_signal = self.left_joystick.getRawButton(3) + -self.left_joystick.getRawButton(2)
        self.arm_motor.set(0.3 * arm_signal)

        #Handle piston in and out
        #Right joystick trigger button toggles claw in or out
        if self.right_joystick.getRawButton(1):
            self.x_pressed_last = True
        elif self.x_pressed_last:
            self.x_pressed_last = False
            self.claw_state = not self.claw_state
        self.set_claw()

        #Reset winch encoder value to 0 if right button 7 is pressed
        if self.right_joystick.getRawButton(7):
            self.winch_encoder.reset()

        #If the right joystick slider is down, go to test mode
        if self.right_joystick.getRawAxis(2) > 0.5:
            self.testMode()


    def testPeriodic():
        pass

    #Test Mode
    def testMode(self):
        #Calculate x and y distance travelled using accelerometer
        a_x = self.accel.getX()
        self.a_x_sum += a_x
        self.a_x_count += 1
        a_y = self.accel.getY()
        self.a_y_sum += a_y
        self.a_y_count += 1

        #Prints acceleration values when right button 8 is pressed
        if self.right_joystick.getRawButton(8):
            print ('x sum: ', self.a_x_sum, ' x count: ', self.a_x_count)
            print ('y sum: ', self.a_y_sum, ' y count: ', self.a_y_count)

        #Resets accelerometer counts when right button 9 is pressed
        if self.right_joystick.getRawButton(9):
            self.a_x_sum = 0.0
            self.a_x_count = 0
            self.a_y_sum = 0.0
            self.a_y_count = 0

        #Prints left ultrasonic sensor values when left button 6 is pressed
        if self.left_joystick.getRawButton(6):
            print ("left_ultrasonic_sensor: ", self.left_ultrasonic_sensor.getValue())
            print ("left limit: ", self.left_claw_whisker())
            print ("right limit: ", self.right_claw_whisker())

        #Prints right ultrasonic sensor values when left button 7 is pressed
        if self.left_joystick.getRawButton(7):
           print ("right_ultrasonic_sensor: ", self.right_ultrasonic_sensor.getValue())

        #Prints left optical sensor values when left button 11 is pressed
        if self.left_joystick.getRawButton(11):
           print ("left_optical_sensor: ", self.right_optical_sensor.get())

        #Prints right optical sensor values when left button 10 is pressed
        if self.left_joystick.getRawButton(10):
           print ("right_optical_sensor: ", self.right_optical_sensor.get())

        #Print current winch encoder value if right button 6 is pressed
        if self.right_joystick.getRawButton(6):
            revs = -self.winch_encoder.get()
            print ('revs: ', revs)


        #Print current gyro value if left button 8 is pressed
        if self.left_joystick.getRawButton(8):
            angle = self.gyro.getAngle()
            print ('angle: ', angle)

        #Reset gyro to 0 if left button 9 is pressed
        if self.left_joystick.getRawButton(9):
            self.gyro.reset()

    #Disabled Mode
    def disabledPeriodic(self):
        self.compressor.stop()

    #Helper Functions
    def set_claw(self):
        self.solenoid1.set(not self.claw_state)
        self.solenoid2.set(self.claw_state)

    def claw_up(self):
        self.claw_state = False

    def claw_down(self):
        self.claw_state = True

    def left_claw_whisker(self):
        return self.left_limit_switch.get()

    def right_claw_whisker(self):
        return self.right_limit_switch.get()

    def winch_set(self, winch_signal):
        """
        Set winch controller safely by taking max and min encoder values into account
        (unless you're pressing the override button)

        winch_signal=0 -> maintain winch position
        winch_signal>0 -> winch up?
        winch_signal<0 -> winch down?
        """
        revs = -self.winch_encoder.get()
        if self.last_winch_signal != 0 and winch_signal == 0:
            self.winch_setpoint = revs
            print ('new setpoint: ', self.winch_setpoint)
        #Right joystick 6 allows encoder override
        if winch_signal == 0:
            #self.winch_motor.set(0.1)
            val = 0.1 + (-0.01 * (revs - self.winch_setpoint))
            print (val)
            self.winch_motor.set(val)
        else:
            if not (self.right_joystick.getRawButton(6)):
                if winch_signal > 0.1 and revs >= 1170:
                    winch_signal = 0
                if winch_signal < -0.1 and revs <= 8:
                    winch_signal = 0
            val = 0.5 * winch_signal
            print (val)
            self.winch_motor.set(val)
        self.last_winch_signal = winch_signal


if __name__ == "__main__":
    wpilib.run(Robot)
