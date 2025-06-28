from formes2d import *
import copy
from Geo import *

class Polygone(Forme2D):
    def __init__(self, sommets=None):
        if sommets is None:
            sommets = []
        self.sommets = sommets
        self.n       = len(sommets)
        self.xdata   = [p.x for p in self.sommets]
        self.ydata   = [p.y for p in self.sommets]
        self.xdata.append(sommets[0].x)
        self.ydata.append(sommets[0].y)

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
        plt.plot(self.xdata, self.ydata)
        if fill:
            plt.fill(self.xdata, self.ydata, 'yellow', alpha=0.30)          
        #for p in self.sommets:
        #    p.draw(dx, dy)
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
        self.xdata  = [p.x for p in self.sommets] # La liste des abscisses
        self.ydata  = [p.y for p in self.sommets] # La liste des ordonnées
        self.xdata.append(self.xdata[0])
        self.ydata.append(self.ydata[0])

    def inclinaison(self, theta):
        return
    
    def translation(self,v):
        ''' Renvoie un autre polygone '''
        return Polygone(*[Point.translate(p,v) for p in self.sommets])

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
        self.xdata = [p.x for p in self.sommets]
        self.ydata = [p.y for p in self.sommets]

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


        
        
        
    