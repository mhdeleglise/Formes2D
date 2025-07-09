from math import cos as cos, sin as sin, sqrt as sqrt, acos as acos, pi as pi, degrees as degrees
import numpy as np
import matplotlib.pyplot as plt

#######################################################################################
def signe(x):
    if x >=0:
        return '+'
    return '-'

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

    def unit(self):
        r = sqrt(self.x**2 + self.y**2)
        return Vecteur(self.x/r, self.y/r)

    def angle(self,w):
        """ angle orienté vers w """
        u,v = self.unit(), w.unit()
        a = acos(u*v)
        if u^v > 0:
            return a
        else:
            return 2*pi-a
        
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

    def __sub__(self, B):
        """ Le vecteur B-self """
        return Vecteur(self.x - B.x, self.y - B.y)

    def draw(self, ax, dx=0.0, dy=0.0, s = 10, color=None, namecolor='black', **kwds):
        plt.scatter(self.x, self.y, s, color, **kwds)
        if not self.nom is None:
            ax.text(self.x + dx, self.y + dy, self.nom, color=namecolor, clip_on=True)
        # if clipon_on == True text n'affiche pas en dehors des axes (ce qu'il fait par défaut

    def rename(self,nom):
        self.nom=nom
    
    def coords(self):
        return (self.x, self.y)        
    
    def distance(self,other):
        v = vecteur(self,other)
        return sqrt(v*v)

    def translate(self, V):
        ''' self est modifié '''
        self.x += V.x
        self.y += V.y

    def symetrie_centrale(self, other):
        if isinstance(other, Point):
            return self + (self - other)
        if isinstance(other, Droite):
            a, b, u,v, w = self.x, self.x, other.a, other.b, other.c
            return Droite(-u, -v, w + 2*u*a + 2*v*b)
        else:
            raise TypeError("La symétrie centrale n'est implémentée que pour un Point ou une Droite")

def barycentre(listePoints, listeCoeffs=[], nom=None):
    """ Si la liste des coefficients est vide, on leur attribue la valeur 1 """
    n = len(listePoints)
    assert n > 0
    if len(listeCoeffs) == 0:
        listeCoeffs = n * [1]
    assert len(listePoints) == len(listeCoeffs)
    sommeCoeffs = sum(listeCoeffs)
    assert sommeCoeffs != 0
    G = Point(0,0,nom)
    for i in range(n):
        G += listeCoeffs[i]*listePoints[i]
    G /= sommeCoeffs
    return G


PointO = Point(0,0)
def milieu(A,B,nom=None):
    assert isinstance(A,Point) and  isinstance(B,Point)
    res = (A+B)/2
    res.nom = nom
    return res
#######################################################################################


class Droite():
    def __init__(self, a, b, c):
        """ La droite d'équation ax + by + c = 0 """
        r = sqrt(a*a + b*b)  
        self.a, self.b, self.c = a/r, b/r, c/r         
            
    def __repr__(self):
        a, b, c = self.a, self.b, self.c
        if self.b ==  0:
            """ a n'est pas nul et x = -c/a """
            return "Droite x = {} {:.4f}".format(signe(-c/a), abs(c/a)) 
        else:
            """ y = -c/b - a/b * x """
            return "Droite y = {} {:.4f} x {} {:.4f}".format(signe(-a/b), abs(a/b), signe(-c/b), abs(c/b))

    def str(self):
        return self.__repr__()[7:]

    def unit(self):
        if self.b > 0:
            return Vecteur(self.b, -self.a)
        else:
            return Vecteur(-self.b, self.a)
    
    def draw(self,ax,**others):
        xmin, xmax = plt.xlim()
        ymin, ymax = plt.ylim()
        #print(xmin, xmax, ymin, ymax)
        ax.set_autoscale_on(False)
        if self.b != 0:
            y0, y1  = [-self.c/self.b - self.a /self.b * x for x in [xmin, xmax]]
            ax.plot([xmin,xmax],[y0,y1], **others)
        else:
            x = -self.c / self.a
            ax.plot([x,x], [ymin,ymax])
            y_vals = [ymin, ymax]
            ax.plot([x, x], y_vals, **others)

    def angle(self, d):
        """ angle orienté de self vers d """
        alpha = self.unit().angle(d.unit())
        if alpha <= pi:
            return alpha
        else:
            return alpha-pi
    
    def parallele(self,P):
        """ La parallèle à self passant par le point P """
        a, b, x, y = self.a, self.b, P.x, P.y
        return Droite(a, b, -(a*x+b*y))

    def orthogonale(self,P):
        """ La perpendiculaire à self passant par le point P """
        a, b, x, y = self.a, self.b, P.x, P.y   
        return Droite(-b, a, b*x - a*y)

    def retournement(self, obj):
        a, b, c = self.a, self.b, self.c
        if isinstance(obj, Point):
            x, y = obj.x, obj.y
            t= -2*(a*x + b*y + c)/(a*a + b*b)
            return Point(x + t*a, y + t*b)    
        if isinstance(obj, Droite):
            u, v, w = obj.a, obj.b, obj.c
            t = -2*(a*u+b*v)/(a**2 + b**2)
            return Droite(u + t*a, v + t*b, w + t*c)         

    def intersection(self,d, nom=None):
        a, b, c = self.a, self.b, self.c
        u, v, w = d.a, d.b, d.c
        delta = a*v - b*u
        dx    = -c*v + b*w
        dy    = -a*w + c*u
        if abs(delta) < 1e-12:
            return None
        else:
            return Point(dx/delta, dy/delta,nom)

    def rotation(self, theta, A=PointO):
        """ Rotation de self, angle theta, autour de A """
        a, b, c = self.a, self.b, self.c
        x0, y0 = A.x, A.y
        u = a*cos(theta) - b*sin(theta)
        v = a*sin(theta) + b*cos(theta)
        w = a*x0 + b*y0 + c - u * x0 - v *y0
        return Droite(u,v,w)
        
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


def vecteur(A,B):
    """ C'est la même chose que B-A """
    assert isinstance(A,Point) and isinstance(B, Point)
    return Vecteur(B.x-A.x, B.y-A.y)

def droite(A,B):
    if not isinstance(A,Point) or not isinstance(B,Point):
        raise TypeError("droite(A,B): A,B ne sont pas deux points")
    else:
        x0, y0, x1, y1 = A.x, A.y, B.x, B.y
        return Droite(y0-y1, x1 - x0, x0*y1 - x1*y0)


def rotation(B,theta,A=PointO):
    ''' Renvoie l'image de u par la rotation d'angle theta autour de A '''
    return Point(A.x + (B.x-A.x) * cos(theta) - (B.y-A.y) * sin(theta), A.y + (B.x-A.x) * sin(theta) + (B.y-A.y) * cos(theta))   

 ##########################################################################################

def symetrie(t, p, nom = None):
    """ symétries centrale de centre p """
    a, b = p.x, p.y
    if isinstance(t,Point):
        res = p + (p - t)
        res.nom = nom
        return res
    if isinstance(t,Droite):
        u,v,w =t.a, t.b, t.c
        return Droite(-u, -v, w + 2*u*a + 2*v*b)

def retournement(obj, d, nom=None):
    """ Les retournements autour de d """
    a, b, c = d.a, d.b, d.c
    if isinstance(obj, Droite):
        u,v,w =  obj.a, obj.b, obj.c
        t = -2*(a*u+b*v)/(a**2 + b**2)
        return Droite(u + t*a, v + t*b, w + t*c)
    if isinstance(obj, Point):
        x, y = obj.x, obj. y
        t= -2*(a*x + b*y + c)/(a*a + b*b)
        return Point(x + t*a, y + t*b, nom)

def projection(P, d, nom = None):
        """ Renvoie la projection orthogonale de P sur d """
        a, b, c = d.a, d.b, d.c
        lbd = -(a * P.x + b * P.y + c)/(a*a + b*b)
        return Point(P.x + lbd * a, P.y + lbd * b, nom)

def parallele(d, P):
    """ La parallèle à d passant par P """
    a,b,x,y = d.a, d.b, P.x, P.y
    return Droite(a, b, -(a*x+b*y))

def orthogonale(d, P):
    """ La perpendiculaire à d passant par P """
    a, b, x, y = d.a, d.b, P.x, P.y   
    return Droite(-b, a, b*x - a*y)    

def mediatrice(A,B):
    assert isinstance(A,Point) and isinstance(B,Point)
    return orthogonale(droite(A,B),(A+B)/2)
    
    
     