from math import cos as cos, sin as sin, sqrt as sqrt, pi as pi
import numpy as np
import matplotlib.pyplot as plt

#######################################################################################

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

#######################################################################################
        
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
    
    def distance(self,other):
        v = vecteur(self,other)
        return sqrt(v*v)

    def translate(self, V):
        ''' self est modifié '''
        self.x += V.x
        self.y += V.y


#######################################################################################

class Droite():
    
    def signe(x):
        if x >=0:
            return '+'
        return '-'
    
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
        """ La parallèle à self passant par le point P """
        a, b, x, y = self.a, self.b, P.x, P.y
        return Droite(a, b, -(a*x+b*y))

    def orthogonale(self,P):
        """ La perpendiculaire à self passant par le point P """
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


PointO = Point(0,0)

def vecteur(A,B):
    assert isinstance(A,Point) and isinstance(B, Point)
    return Vecteur(B.x-A.x, B.y-A.y)


def rotation(B,theta,A=PointO):
    ''' Renvoie l'image de u par la rotation d'angle theta autour de A '''
    return Point(A.x + (B.x-A.x) * cos(theta) - (B.y-A.y) * sin(theta), A.y + (B.x-A.x) * sin(theta) + (B.y-A.y) * cos(theta))   

    
     