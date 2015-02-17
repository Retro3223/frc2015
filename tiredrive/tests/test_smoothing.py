from tiredrive.robot import Smooth


def test_smooth():
    smoothie = Smooth(0, 0.1)
    for i in range(5):
        assert .1 * (i+1) == smoothie.set(0.5)
