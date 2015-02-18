class XboxController:
    def __init__(self, joystick):
        self.joystick = joystick

    def A(self):
        return self.joystick.getRawButton(1)

    def B(self):
        return self.joystick.getRawButton(2)

    def X(self):
        return self.joystick.getRawButton(3)

    def Y(self):
        return self.joystick.getRawButton(4)

    def left_bump(self):
        return self.joystick.getRawButton(5)

    def right_bump(self):
        return self.joystick.getRawButton(6)

    def start(self):
        return self.joystick.getRawButton(8)

    def back(self):
        return self.joystick.getRawButton(7)

    def left_joystick_down(self):
        return self.joystick.getRawButton(9)

    def right_joystick_down(self):
        return self.joystick.getRawButton(10)

    def left_joystick_axis_v(self):
        return self.joystick.getRawAxis(1)

    def left_joystick_axis_h(self):
        return self.joystick.getRawAxis(0)

    def right_joystick_axis_h(self):
        return self.joystick.getRawAxis(4)

    def right_joystick_axis_v(self):
        return self.joystick.getRawAxis(5)

    def right_trigger(self):
        return self.joystick.getRawAxis(3)

    def left_trigger(self):
        return self.joystick.getRawAxis(2)

    def d_pad(self):
        return self.joystick.getPOV()
