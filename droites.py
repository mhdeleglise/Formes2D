from formes2d import *
import matplotlib.pyplot as plt

class Droite():
    def __init__(self, a, b, c):
        """ La droite d'équation ax + by + c = 0 """
        self.a = a
        self.b = b
        self.c = c

    def draw(self,ax):
        if self.b != 0:
            x0, x1 = ax.get_xlim()
            x0 -= 0.05
            x1 += 0.05
            y0, y1  = [self.c - (self.a * x  + self.c)/self.b for x in [x0, x1]]
            ax.plot([x0,x1],[y0,y1])
        else:
            x = -self.c/self.a
            ax.axvline(x=x)

            
        
        
