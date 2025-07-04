#from formes2d import *
import matplotlib.pyplot as plt
import numpy as np
from Geo import *

def signe(x):
    if x >=0:
        return '+'
    return '-'

class Droite():
    def __init__(self, a, b, c):
        """ La droite d'équation ax + by + c = 0 """
        if b == 0:
            self.a, self.b, self.c =   1, 0, c/a
        else:  
            self.a, self.b, self.c = a/b, 1, c/b         
            
    def __repr__(self):
        a,c = self.a, self.c
        if self.b ==  0:
            return "Droite x = {} {:.3f}".format(signe(-c), abs(c)) 
        else:
            return "Droite y = {} {:.3f} x {} {:.3f}".format(signe(-a), abs(a), signe(-c), abs(c))

    def str(self):
        return self.__repr__()[7:]
        
    def draw(self,ax,**others):
        xmin, xmax = plt.xlim()
        ymin, ymax = plt.ylim()
        if self.b == 1:
            y0, y1  = [-self.c - self.a * x for x in [xmin, xmax]]
            ax.plot([xmin,xmax],[y0,y1], **others)
        else:
            x = -self.c
            ax.plot([x,x], [ymin,ymax])
            y_vals = [ymin, ymax]
            ax.plot([x, x], y_vals, **others)

    
    def parallele(self,P):
        """ La parallèle à self passant par P """
        a, b, x, y = self.a, self.b, P.x, P.y
        return Droite(a, b, -(a*x+b*y))

    def orthogonale(self,P):
        """ La perpendiculaire à self passant par P """
        a, b, x, y = self.a, self.b, P.x, P.y   
        return Droite(-b, a, b*x - a*y)


class DemiDroite():
    def __init__(self,A,V):
        self.A = A
        self.V = V

    def draw(self,ax,**kwds):
        xA, yA, u, v = self.A.x, self.A.y, self.V[0], self.V[1]
        xmin, xmax = plt.xlim()
        if u > 0:
            yend = yA  + (xmax-xA)*v
            ax.plot([xA, xmax], [yA, yend], **kwds)
        else:
            ystart = yA - (xA-xmin)*v
            ax.plot([xmin, xA], [ystart, yA], **kwds)
            

def droite(p, v):
    """ Droite définie par un point p et un vecteur v """
    x0, y0 = p
    a,  b  = v
    return Droite(b,-a,b*x0-a*y0)
    
    
class Segment():
    def __init__(self, A, B):
        self.A = A
        self.B = B

    def __repr__(self):
        return f"Segment({self.A}, {self.B})"        
    
    def draw(self, ax, **kwds):
        ax.plot([self.A.x,self.B.x],[self.A.y,self.B.y],**kwds)
        #self.A.draw(ax)
        #self.B.draw(ax)
    
        
        
