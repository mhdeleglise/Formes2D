from formes2d import *
import copy
from Geo import *

class Polygone(Forme2D):
    def __init__(self, sommets=None):
        if sommets is None:
            sommets = []
        self.sommets = sommets
        self.n       = len(sommets)
 
    def __repr__(self):
        res = ''
        for p in self.sommets:
            res += str(p) + ', '
        return 'Polygone(' + res[:-2] + ')'
    
    def add_sommet(self, point):
        self.sommets.append(point)
        self.n += 1

    def draw(self, dx=0.15, dy=0.15, fill=False):
        plt.axis('equal')
        xdata   = [p.x for p in self.sommets]
        ydata   = [p.y for p in self.sommets]
        xdata.append(self.sommets[0].x)
        ydata.append(self.sommets[0].y)
        plt.plot(xdata, ydata)
        if fill:
            plt.fill(xdata, ydata, 'yellow', alpha=0.30)          
        for p in self.sommets:
            p.draw(dx, dy)
        return

    def coords(self,i):
        ''' Les coordonnées du ième sommet '''
        return self.sommets[i].coords()

    def rotation(self, theta, A=O):
        ''' Renvoie un autre polygone '''
        return Polygone([rotation(u,theta,A) for u in self.sommets])

    def rotate(self, theta):
        ''' Rotation en place '''
        for u in self.sommets:
            u = u.rotate(theta)

    def translate(self, dx, dy):
        ''' Renvoie un autre polygone '''
        for p in self.sommets:
            p.translate(dx,dy)
        #return Polygone(*[Point.translate(p,v) for p in self.sommets])

    def perimetre(self):
        n = len(self.sommets)
        p = self.sommets
        return p[0].distance(p[-1]) + sum ([p[i].distance(p[i+1]) for i in range(n-1)]) 

    def aire(self):
        res = 0.0
        for i in range(self.n-2):
            a = Triangle(self.coords(0), self.coords(i+1), self.coords(i+2)).aire()
            res += a
        return res

    def isobarycentre(self):
        gres = Point(0,0)
        s = 0
        pts = []
        aires = []
        for i in range(self.n-2):
            tr = Triangle(self.coords(0), self.coords(i+1), self.coords(i+2))
            a  = tr.aire()
            g  = tr.centre()
            s += a
            gres = gres + Point.times(g,a)
        return Point.times(gres,1/s)
            
class Triangle(Polygone): 
    def __init__(self, A, B, C):
        self.size = 3
        self.sommets=[Point(*A), Point(*B),Point(*C)]
 
    def __repr__(self):
        return 'Triangle' + str(self.sommets)

    def aire(self):
        u = Point.vecteur(self.sommets[0],self.sommets[1])
        v = Point.vecteur(self.sommets[0],self.sommets[2])
        return abs(Vecteur.det(u,v))/2

    def centre(self):
        u = self.sommets[0]
        v = self.sommets[1]
        w = self.sommets[2]
        return Point((u.x + v.x + w.x)/3, (u.y + v.y + w.y)/3)


        
        
        
    