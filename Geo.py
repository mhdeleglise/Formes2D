from math import cos as cos, sin as sin, sqrt as sqrt, pi as pi
import numpy as np
import matplotlib.pyplot as plt
from Droites import *

class Vecteur:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y  

    def __repr__(self):
        return f"Vecteur({self.x},{self.y})"      
    
    def __mul__(u,t):
        if isinstance(t,Vecteur):
            return u.x * t.x +  u.y * t.y
        elif isinstance(t, (int, float)):
            return Vecteur(t*(u.x),t*(u.y))
        else:
            raise TypeError("Un vecteur ne peut être multiplié que par un nombre où un autre vecteur")

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __xor__(self,v):
        ''' L'opérateur xor ^ est comme le produit vectoriel '''
        return self.x * v.y - v.x * self.y

    def det(u,v):
        ''' Le determinant , même chose que le précédent ''' 
        return u.x * v.y - v.x * u.y

class Point: #Espace affine euclidien de dimension 2
    
    def __init__(self, x=0, y=0, nom=None):
        self.nom = nom
        self.x = x
        self.y = y  

    def __repr__(self):
        return f"Point({self.x},{self.y},{self.nom})"
        
    def __add__(self,other):
        """ l'opérateur + devient l'addition de Points """
        return Point(self.x + other.x, self.y + other.y)

    def __iadd__(self,other):
        return self.__add__(other)

    def __mul__(self, t, name=None):
        if not isinstance(t, (int, float)):
            raise TypeError("Un point ne être multiplié que par un scalaire")
        else:
            return Point(t*self.x, t*self.y)
            
    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        return Point(self.x / scalar, self.y / scalar)

    def __sub__(self, other):
        """ La différence A - B de deux points A et B est le vecteur de la translation qui envoie A sur B """
        return Vecteur(self.x-other.x, self.y - other.y)

    def draw(self, ax, dx=0.05, dy=0.0, s = 10, color=None, namecolor='black', **kwds):
        plt.scatter(self.x, self.y, s, color, **kwds)
        if not self.nom is None:
            ax.text(self.x + dx, self.y + dy, self.nom, color=namecolor, clip_on=True)
        # if clipon_on == True text n'affiche pas en dehors des axes (ce qu'il fait par défaut

    def coords(self):
        return (self.x, self.y)        
    
    def vecteur(self,other):
        return Vecteur(self.x-other.x, self.y - other.y)

    def distance(self,other):
        v = Point.vecteur(self,other)
        return sqrt(v*v)

    def rotate(self, theta):
        ''' Rotation en place le point est déplacé '''
        x, y = self.x, self.y
        self.x = x * cos(theta) - y * sin(theta)
        self.y = x * sin(theta) + y * cos(theta)
    
    def translate(self, V):
        ''' self est modifié '''
        self.x += V.x
        self.y += V.y

PointO = Point(0,0)

def rotation(B,theta,A=PointO):
    ''' Renvoie l'image de u par la rotation d'angle theta autour de A '''
    return Point(A.x + (B.x-A.x) * cos(theta) - (B.y-A.y) * sin(theta), A.y + (B.x-A.x) * sin(theta) + (B.y-A.y) * cos(theta))   

    
     