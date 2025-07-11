import matplotlib.pyplot as plt
from Geo2d import *


class Cercle():
    def __init__(self, centre, r):
        self.centre = centre
        self.rayon = r

    def draw(self, ax, color=None):
        theta = np.linspace(0, 2*np.pi, 100)
        x1 = self.centre.x + self.rayon*np.cos(theta)
        x2 = self.centre.y + self.rayon*np.sin(theta)
        ax.plot(x1, x2, color=color)
        ax.set_aspect(1)

    def aire(self):
        return math.pi * self.rayon ** 2

    def perimetre(self):
        return 2 * math.pi * self.rayon

    def translate(self, v):
        self.centre.translate(v)

    def rotation(self, angle_degres, centre=None):
        self.centre.rotation(angle_degres, centre)
        