from formes2d import *
import matplotlib.pyplot as plt

class Droite():
    def __init__(self, a, b, c):
        """ La droite d'équation ax + by = c """
        if b != 0:
            self.a, self.b, self.c = a/b, 1, -c/b
        else:
            self.a, self.b, self.c =   1, 0, -c/b
            
    def str(self):
        print("Droite( ax+by avec a,b,c) = ",self.a,self.b,self.c)
    
    def draw(self,ax):
        if self.b == 1:
            x0, x1 = ax.get_xlim()
            print("x0,x1= ", x0,x1)
            y0, y1  = [-self.c - self.a * x for x in [x0, x1]]
            ax.plot([x0,x1],[y0,y1])
        else:
            x = -self.c
            ax.axvline(x=x)

class DemiDroite():
    def __init__(self,A,V):
        self.A = A
        self.V = V

    def draw(self,ax):
        xA, yA, u, v = self.A.x, self.A.y, self.V[0], self.V[1]
        xmin, xmax = plt.xlim()
        if u > 0:
            yend = yA  + (xmax-xA)*v
            ax.plot([xA, xmax], [yA, yend])
        else:
            ystart = yA - (xA-xmin)*v
            ax.plot([xmin, xA], [ystart,yA])
            

def droite(p, v):
    """ Droite définie par un point p et un vecteur v """
    x0, y0 = p
    a, b   = v
    return Droite(b,-a,b*x0-a*y0)
    
    
class Segment():
    def __init__(self, A, B):
        self.A = A
        self.B = B

    def __repr__(self):
        return f"Point({self.A}, {self.B})"        
    
    def draw(self, ax):
        ax.plot([self.A.x,self.B.x],[self.A.y,self.B.y])
    
        
        
