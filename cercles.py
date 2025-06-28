from formes2d import *
import matplotlib.pyplot as plt

class Cercle(Forme2D):
    def __init__(self, centre, rayon):
        self.centre = centre
        self.rayon = rayon

    def aire(self):
        return math.pi * self.rayon ** 2

    def perimetre(self):
        return 2 * math.pi * self.rayon

    def draw(self, ax):
        cercle = plt.Circle((self.centre.x, self.centre.y), self.rayon, fill=False, color='green')
        ax.add_patch(cercle)

    def translate(self, dx, dy):
        self.centre.translate(dx, dy)

    def rotation(self, angle_degres, centre=None):
        self.centre.rotation(angle_degres, centre)