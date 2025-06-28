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

    def det(u,v):
        ''' Le determinant ''' 
        return v.x * u.y - u.x * v.y

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
            

    def draw(self, ax, dx=0.0, dy=0.0, s=20, color= 'blue', textcolor = 'blue'):
        plt.scatter(self.x, self.y,  s=s)
        if not self.nom is None:
            plt.text(self.x + dx, self.y+dy, self.nom, color=textcolor)

    def coords(self):
        return (self.x, self.y)
        
    def __add__(self,other):
        return Point(self.x + other.x, self.y + other.y)

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

    def translate(self, dx, dy):
        self.x += dx
        self.y += dy

O = Point(0,0)

def rotation(u,theta,A=O):
    ''' Renvoie l'image de u par la rotation d'angle theta autour de (0,0) '''
    return Point(A.x + (u.x-A.x) * cos(theta) - (u.y-A.y) * sin(theta), A.y + (u.x-A.x) * sin(theta) + (u.y-A.y) * cos(theta))   

    
     