import matplotlib.pyplot as plt
from Geo2d import *


class Cercle():
    def __init__(self, centre, r):
        self.centre = centre
        self.rayon = r

    def __repr__(self):
        return f"Cercle, centre:{self.centre}, rayon:{self.rayon:.3f}"
        
    def draw(self, ax, color=None, facecolor=None,**kwds):
        theta = np.linspace(0, 2*np.pi, 100)
        x1 = self.centre.x + self.rayon*np.cos(theta)
        x2 = self.centre.y + self.rayon*np.sin(theta)
        ax.plot(x1, x2, color=color,**kwds)
        if not facecolor is None:
            theta = np.linspace(0, np.pi, 100)
            x = self.centre.x + self.rayon*np.cos(theta)
            y1 = self.centre.y - self.rayon*np.sin(theta)
            y2 = self.centre.y + self.rayon*np.sin(theta)            
            plt.fill_between(x,y1,y2, color = facecolor)
        ax.set_aspect(1)

    def aire(self):
        return math.pi * self.rayon ** 2

    def perimetre(self):
        return 2 * math.pi * self.rayon

    def translate(self, v):
        self.centre.translate(v)

    def rotation(self, angle_degres, centre=None):
        self.centre.rotation(angle_degres, centre)

    def cercle(A,B,C):
        mAB = mediatrice(A,B)
        mAC = mediatrice(A,C)
        centre = mAB.intersection(mAC)
        return Cercle(centre,A.distance(centre))
        