import matplotlib.pyplot as plt
from Geo2d import *

class Cercle():
    def __init__(self, centre, rayon):
        self.centre = centre
        self.rayon = rayon

    def aire(self):
        return math.pi * self.rayon ** 2

    def perimetre(self):
        return 2 * math.pi * self.rayon

    def draw(self, ax, fill=True, **kwds):
        cercle = plt.Circle((self.centre.x, self.centre.y), self.rayon, fill=fill, color='green')
        ax.add_patch(cercle)

    def translate(self, v):
        self.centre.translate(v)

    def rotation(self, angle_degres, centre=None):
        self.centre.rotation(angle_degres, centre)
