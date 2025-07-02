from math import cos as cos, sin as sin, sqrt as sqrt, pi as pi
import numpy as np
import matplotlib.pyplot as plt

class Vecteur:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y  

    def __mul__(u,v):
        ''' produit scalaire'''
        return u.x * v.x + u.y * v.y

    def __xor__(self,v):
        ''' Le produit vectoriel '''
        return self.x * v.y - v.x * self.y

    def det(u,v):
        ''' Le determinant , même chose que le précédent ''' 
        return u.x * v.y - v.x * u.y

class Point: # Espace affine euclidien de dimension 2
    
    def __init__(self, x=0, y=0, nom=None):
        self.nom = nom
        self.x = x
        self.y = y  

    def __repr__(self):
        if self.nom is None:
            return f"({self.x},{self.y})"
        else:
            return f"{self.nom}({self.x},{self.y})"
    
    def draw(self, ax, dx=0.05, dy=0.0, s = 10, color=None, namecolor='black', **kwds):
        print("In Points.draw with a name: ", self.x, self.y, dx, dy)
        print("kwds:  ")
        for u, v in kwds.items():
            print(u, ' = ' , v)
        plt.scatter(self.x, self.y, s, color, **kwds),
        if not self.nom is None:
            print("x+dx= ", self.x+dx, "  y+dy= ", self.y + dy)
            ax.text(self.x + dx, self.y + dy, self.nom, color=namecolor, clip_on=True)
        # if clipon_on == True text n'affiche pas en dehors des axes (ce qu'il fait
        # par défaut

    def coords(self):
        return (self.x, self.y)
        
    def __add__(self,other):
        return Point(self.x + other.x, self.y + other.y)

    def __iadd__(self,other):
        return self.__add__(other)

    def __mul__(self, scalar, name=None):
        return Point(self.x * scalar, self.y * scalar, name)

    def __rmul__(self, scalar, nom=""):
        return self.__mul__(scalar, nom)  # permet d’écrire 0.5 * A

    def __truediv__(self, scalar):
        return Point(self.x / scalar, self.y / scalar)

    def __sub__(self, other):
        """ La différence A - B de deux points A et B est le vecteur de la translation qui envoie A sur B """
        return Vecteur(self.x-other.x, self.y - other.y)

    def vecteur(self,other):
        return Vecteur(self.x-other.x, self.y - other.y)

    def distance(self,other):
        v = Point.vecteur(self,other)
        return sqrt(v*v)

    def times(self, a):
        ''' a est un réel, renvoie a * self '''
        return Point(a * self.x, a * self.y)

    def rotate(self, theta):
        ''' Rotation en place le point est déplacé '''
        x, y = self.x, self.y
        self.x = x * cos(theta) - y * sin(theta)
        self.y = x * sin(theta) + y * cos(theta)
    
    #def rotation(u,theta,x=0,y=0):
    #    ''' Renvoie l'image de self par la rotation d'angle theta autour de (0,0) '''
    #    return Point(x + (u.x-x) * cos(theta) - (u.y-y) * sin(theta), y + (u.x-x) * sin(theta) + (u.y-y) * cos(theta))   

    def translate(self, V):
        ''' self est modifié '''
        self.x += V.x
        self.y += V.y

def tracer_point(ax, p: Point, couleur='blue', **kwds):
        ax.plot(p.x, p.y, 'o', color=couleur, **kwds)
        if p.nom:
            ax.text(p.x + 0.2, p.y + 0.1, p.nom, fontsize=12)           


O = Point(0,0)

def rotation(u,theta,A=O):
    ''' Renvoie l'image de u par la rotation d'angle theta autour de (0,0) '''
    return Point(A.x + (u.x-A.x) * cos(theta) - (u.y-A.y) * sin(theta), A.y + (u.x-A.x) * sin(theta) + (u.y-A.y) * cos(theta))   

    
     