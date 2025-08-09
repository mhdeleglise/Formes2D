from math import cos as cos, sin as sin, sqrt as sqrt, acos as acos, pi as pi, degrees as degrees
from intersection import *
import numpy as np
import matplotlib.pyplot as plt

#######################################################################################
def signe(x):
    if x >=0:
        return '+'
    return '-'

class Vecteur:
    def __init__(self,x:float =0,y:float =0):
        self.x = x
        self.y = y  

    def __repr__(self):
        return f"Vecteur({self.x},{self.y})"      

    def __add__(u,t):
        return Vecteur(u.x + t.x, u.y + t.y)

    def __truediv__(self, scalar):
        return Vecteur(self.x / scalar, self.y / scalar)

    def __eq__self(other):
        return self.x == other.x and self.y == other.y
    
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

    def normalize(self):
        """ Transforme ce vecteur en un vecteur unitaire (cad. de longueur 1) """
        r = sqrt(self.x**2 + self.y**2)
        self.x /= r
        self.y /= r
    
    def unit(self):
        """ Renvoie le vecteur unitaire correspondant, self n'est pas modifié """
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

def det(u: Vecteur, v: Vecteur):
    ''' Le determinant, même chose que le précédent ''' 
    return u.x * v.y - v.x * u.y
#######################################################################################
        
class Point: #Espace affine euclidien de dimension 2
    
    def __init__(self, x:float =0, y: float =0, nom=None):
        self.nom = nom
        self.x = x
        self.y = y  

    def __repr__(self):
        return f"Point({self.x:.2f},{self.y:.2f},{self.nom})"
        
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

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __sub__(self, B):
        """ Le vecteur B-self """
        return Vecteur(self.x - B.x, self.y - B.y)

    def draw(self, ax, dx=0.0, dy=0.0, s = 10, color=None, namecolor='black', **kwds):
        plt.scatter(self.x, self.y, s, color=color, **kwds)
        if not self.nom is None:
            ax.text(self.x + dx, self.y + dy, self.nom, color=namecolor, clip_on=True)
            
    def rename(self,nom):
        self.nom=nom
    
    def coords(self):
        return (self.x, self.y)        
    
    def distance(self,other):
        v = vecteur(self,other)
        return sqrt(v*v)

    def distance(self, other):
        if isinstance(other, Point):
            v = vecteur(self,other)
            return sqrt(v*v)
        else:
            a,b,c = other.a, other.b, other.c
            return abs(a*self.x + b *self.y + c)

    def translate(self, V):
        ''' self est modifié '''
        self.x += V.x
        self.y += V.y

    def translation(self, V, nom=None):
        """ image de self par la translation de vecteur """
        return Point(self.x + V.x, self.y + V.y, nom)

    def symetrie_centrale(self, other):
        if isinstance(other, Point):
            return self + (self - other)
        if isinstance(other, Droite):
            a, b, u,v, w = self.x, self.x, other.a, other.b, other.c
            return Droite(-u, -v, w + 2*u*a + 2*v*b)
        else:
            raise TypeError("La symétrie centrale n'est implémentée que pour un Point ou une Droite")

def barycentre(listePoints : list[Point], listeCoeffs : list[float] =[], nom=None) -> Point:
    """ Si la liste des coefficients est vide, on attribue à chacun d'eux la valeur 1 """
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
    def __init__(self, a: float, b: float, c:float):
        """ La droite d'équation ax + by + c = 0 """
        r = sqrt(a*a + b*b)
        a, b, c = a/r, b/r, c/r
        if a < 0:
            a,b,c = -a,-b,-c
        if a == 0 and b < 0:
            b, c = -b, -c
        (self.a, self.b, self.c) =  a, b, c
        self.vu = -b, a

    def verticale(self):
        return self.b == 0.0
        
    def __repr__(self):
        a, b, c = self.a, self.b, self.c
        if self.b ==  0:
            """ a n'est pas nul et x = -c/a """
            return "Droite verticale x= {self.a:.2f}"
        else:
            """ y = -c/b - a/b * x """
            return f"Droite y = {signe(-a/b)} {abs(a/b):.3f} x {signe(-c/b)} {abs(c/b):.3f}"#.format(signe(-a/b), abs(a/b), signe(-c/b), abs(c/b))
            
    def __eq__(self, other):
        return self.a == other.a and self.b == other.b and  self.c == other.c
    
    def str(self):
        return self.__repr__()[7:]

    def unit(self):
        if self.b > 0:
            return Vecteur(self.b, -self.a)
        else:
            return Vecteur(-self.b, self.a)

    def pointAbscisse(self,x,nom=None):
        assert self.b != 0
        a, b, c = self.a, self.b, self.c
        return Point(x, (-c - a*x)/b,nom)
    
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

    def bissectrice(self, d, n=0):
        """ Il y a deux bissectrices, """
        a, b, c = self.a, self.b, self.c
        u, v, w = d.a, d.b, d.c
        if n == 0:
            return Droite(a-u, b-v, c-w)
        else:
            return Droite(a+u, b+v, c+w)
    
    def parallele(self,P):
        """ La parallèle à self passant par le point P """
        a, b, x, y = self.a, self.b, P.x, P.y
        return Droite(a, b, -(a*x+b*y))

    def orthogonale(self,P: Point):
        """ La perpendiculaire à self passant par le point P """
        a, b, x, y = self.a, self.b, P.x, P.y   
        return Droite(-b, a, b*x - a*y)

    def retournement(self, obj):
        """ L'image par retournement d'un objet, point ou droite, autour de self """
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
        """ Intersection de self avec la droite d """        
        a, b, c = self.a, self.b, self.c
        u, v, w = d.a, d.b, d.c
        delta = b*u -a*v
        dx    = c*v - b*w
        dy    = a*w-c*u
        if abs(delta) < 1e-12:
            return None
        else:
            return Point(dx/delta, dy/delta, nom)

    def rotation(self, theta, A=PointO):
        """ Rotation de self, angle theta, autour de A """
        a, b, c = self.a, self.b, self.c
        x0, y0 = A.x, A.y
        u = a*cos(theta) - b*sin(theta)
        v = a*sin(theta) + b*cos(theta)
        w = a*x0 + b*y0 + c - u * x0 - v *y0
        return Droite(u,v,w)

class DemiDroite(Droite):
    """ (y - y0)/(x-x0) = vy/vx """
    def __init__(self,A,V):
        assert isinstance(A, Point) and isinstance(V, Vecteur)
        V.normalize()
        self.origine = A
        self.V       = V
        x0, y0 = A.x, A.y
        a = V.y; b = -V.x; c = y0 * V.x -x0 * V.y
        super().__init__(a,b,c)
        if V.x == 0:
            self.direction = "up" if V.y > 0 else "down"
        else:
            self.direction = "right" if V.x > 0 else "left"
            self.pente = V.y/V.x
        
    def __repr__(self):
        if not self.verticale():
            return f"Origine: {self.origine}, Pente: {self.pente:.2f} Direction: {self.direction}"
        else:
            return f"Origine: {self.origine}, Verticale,  Direction: {self.direction}"
            
        

    def draw(self, ax,**kwds):
        xA, yA = self.origine.x, self.origine.y
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        match self.direction:
            case 'up':
                ax.plot([xA,xA],[yA,ymax],**kwds)
                return
            case 'down':
                ax.plot([xA,xA],[yA,ymin],**kwds)
                return
            case 'right':
                if self.pente > 0:
                    xmax = min(xmax, xA + (ymax-yA)/self.pente)
                elif self.pente < 0:
                    xmax = min(xmax, xA + (yA-ymin)/abs(self.pente))
                ax.plot([xA,xmax],[yA,yA + (xmax-xA)*self.pente],**kwds)  
                return
            case 'left':
                if self.pente > 0:
                    xmin = max(xmin, xA - (yA-ymin)/self.pente)
                elif self.pente < 0:
                    xmin = max(xmin, xA -(ymax-yA)/abs(self.pente))
                ax.plot([xmin, xA],[yA+(xmin-xA)*self.pente,yA],**kwds)
                return
                    
    def bissectrice(self, d):
        """ La bissectrice de self et d """
        assert isinstance(d, DemiDroite) and d.origine == self.origine
        a,b = self.origine.x, self.origine.y
        return DemiDroite(Point(a,b), (self.V + d.V)/2)

def droite(p: Point, v:Vecteur):
    """ Droite définie par un point p et un vecteur v """
    x0, y0 = p
    a,  b  = v
    return Droite(b,-a,b*x0-a*y0)
    
def demi_droite(A: Point, B: Point):
    """ demi droite définie par un couple de points """
    v = vecteur(A,B)
    return DemiDroite(A, vecteur(A,B))

class Segment():
    def __init__(self, A, B):
        self.A = A
        self.B = B

    def __repr__(self):
        return f"Segment({self.A}, {self.B})"        
    
    def draw(self, ax, **kwds):
        ax.plot([self.A.x,self.B.x],[self.A.y,self.B.y],**kwds)

    def inter(self, u):
        return doIntersect(self.A, self.B, u.A, u.B)

    def __eq__(self,other):
        reurn (self.A == other.A and self.B == other.B) or (self.A == other.B and self.B == other.A)
        
def vecteur(A:Point, B:Point):
    """ C'est la même chose que B-A """
    assert isinstance(A,Point) and isinstance(B, Point)
    return Vecteur(B.x-A.x, B.y-A.y)

def droite(A : Point, B: Point):
    if not isinstance(A,Point) or not isinstance(B,Point):
        raise TypeError("droite(A,B): A,B ne sont pas deux points")
    else:
        x0, y0, x1, y1 = A.x, A.y, B.x, B.y
        return Droite(y0-y1, x1 - x0, x0*y1 - x1*y0)


def rotation(B,theta,A=PointO):
    ''' Renvoie l'image de u par la rotation d'angle theta autour de A '''
    return Point(A.x + (B.x-A.x) * cos(theta) - (B.y-A.y) * sin(theta), A.y + (B.x-A.x) * sin(theta) + (B.y-A.y) * cos(theta))   

 ##########################################################################################

def symetrie(t, p : Point, nom = None):
    """ symétrie centrale de centre p """
    a, b = p.x, p.y
    if isinstance(t,Point):
        res = p + (p - t)
        res.nom = nom
        return res
    if isinstance(t,Droite):
        u,v,w =t.a, t.b, t.c
        return Droite(-u, -v, w + 2*u*a + 2*v*b)

def retournement(obj, d, nom=None):
    """ Retournement d'un point ou d'une droite  autour de la droite d  """
    a, b, c = d.a, d.b, d.c
    if isinstance(obj, Droite):
        u,v,w =  obj.a, obj.b, obj.c
        t = -2*(a*u+b*v)/(a**2 + b**2)
        return Droite(u + t*a, v + t*b, w + t*c)
    if isinstance(obj, Point):
        x, y = obj.x, obj. y
        t= -2*(a*x + b*y + c)/(a*a + b*b)
        return Point(x + t*a, y + t*b, nom)

def projection(P: Point, d:Droite, nom = None):
        """ Renvoie la projection orthogonale de P sur d """
        a, b, c = d.a, d.b, d.c
        lbd = -(a * P.x + b * P.y + c)/(a*a + b*b)
        return Point(P.x + lbd * a, P.y + lbd * b, nom)

def parallele(d: Droite, P: Point):
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
    
    
     