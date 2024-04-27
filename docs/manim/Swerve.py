from manim import *

class SwerveModule(VGroup):
    def __init__(self):
        super().__init__()
        self.circle = Circle(2)
        self.add(self.circle)

class Swerve(VGroup):
    def __init__(self, robot_width: float, drive_width, x: float, y: float, theta: float, vx: float, vy: float, vtheta: float) -> None:
        super().__init__()
        self.square = Square(robot_width)
        self.fl = SwerveModule()
        self.fr = SwerveModule()
        self.bl = SwerveModule()
        self.br = SwerveModule()
        self.add(self.square, self.fl, self.fr, self.bl, self.br)
        

