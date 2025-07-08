import matplotlib.pyplot as plt
import numpy as np

class Cercle():
    def __init__(self, centre, rayon):
        self.centre = centre
        self.rayon = rayon

    def aire(self):
        return math.pi * self.rayon ** 2

    def perimetre(self):
        return 2 * math.pi * self.rayon

    """
    def draw(self, ax):
        cercle = plt.Circle((self.centre.x, self.centre.y), self.rayon, fill=False, color='green')
        ax.add_patch(cercle)
    """

    def draw(self, ax, fill = False, color = 'green',**others):
        x0, y0, r = self.centre.x, self.centre.y, self.rayon
        x  = np.linspace(x0-r, x0 + r,100)
        y1 = y0 + np.sqrt(r**2 - (x-x0)*(x-x0))
        y2 = y0 - np.sqrt(r**2 - (x-x0)*(x-x0))
        plt.axis('equal')
        plt.grid(True)
        if fill:
            plt.fill_between(x,y1,y2, color = color)
        else:
            ax.plot(x,y1,color='green',**others)
            ax.plot(x,y2,color='green')
    
    def translate(self, v):
        self.centre.translate(v)

    def rotation(self, angle_degres, centre=None):
        self.centre.rotation(angle_degres, centre)