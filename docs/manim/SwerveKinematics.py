from manim import *
from Swerve import Swerve

class SwerveKinematics(Scene):
    def construct(self):
        self.add(Swerve(25, 2, 3, 4, 5, 6, 7, 8))