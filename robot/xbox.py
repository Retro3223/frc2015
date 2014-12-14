class XboxController:
    def __init__(self, joystick):
        self.joystick = joystick

    def A(self):
        return self.joystick.GetRawButton(1)

    def B(self):
        return self.joystick.GetRawButton(2)

    def X(self):
        return self.joystick.GetRawButton(3)

    def Y(self):
        return self.joystick.GetRawButton(4)

    def left_bump(self):
        return self.joystick.GetRawButton(5)

    def right_bump(self):
        return self.joystick.GetRawButton(6)

    def start(self):
        return self.joystick.GetRawButton(8)

    def back(self):
        return self.joystick.GetRawButton(7)

    def left_joystick_down(self):
        return self.joystick.GetRawButton(9)

    def right_joystick_down(self):
        return self.joystick.GetRawButton(10)

    def left_joystick_axis_v(self):
        return self.joystick.GetRawAxis(2)

    def left_joystick_axis_h(self):
        return self.joystick.GetRawAxis(1)

    def right_joystick_axis_h(self):
        return self.joystick.GetRawAxis(4)

    def right_joystick_axis_v(self):
        return self.joystick.GetRawAxis(5)

    def right_trigger(self):
        return self.joystick.GetRawAxis(3)

    def left_trigger(self):
        return self.joystick.GetRawAxis(3)
