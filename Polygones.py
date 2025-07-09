from Geo2d import *
from Cercles import Cercle

class Polygone():
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
    
    #def add_sommet(self, point):
    #    self.sommets.append(point)
    #    self.n += 1

    def draw(self, ax, dx=0.0, dy=0.0, color=None, namecolor=None, ms = 0, **kwds):
        xdata   = [p.x for p in self.sommets]
        ydata   = [p.y for p in self.sommets]
        xdata.append(self.sommets[0].x)
        ydata.append(self.sommets[0].y)
        ax.plot(xdata, ydata, '-o', color= color,ms=ms)
        if not color is None:
            plt.fill(xdata, ydata, color, alpha=0.30)  
        return

    def sommet(self,i):
        return self.sommets[i % self.n]

    def has_sommet(self, A):
        return A in self.sommets
    
    def coords(self,i):
        ''' Les coordonnées du ième sommet '''
        return self.sommets[i].coords()

    def rotation(self, theta, A=PointO):
        ''' Renvoie un autre polygone '''
        return Polygone([rotation(u,theta,A) for u in self.sommets])

    def rotate(self, theta, A=0):
        ''' Rotation en place, chaque sommet est remplacé par son imagge
        par la rotation de theta aoutour de A'''
        for u in self.sommets:
            u = u.rotate(theta)

    def translate(self, V):
        ''' Translation de self par le Vecteur V
        tous les sommets de self sont translatés '''
        for p in self.sommets:
            p.translate(V)

    def perimetre(self):
        n = len(self.sommets)
        p = self.sommets
        return p[0].distance(p[-1]) + sum ([p[i].distance(p[i+1]) for i in range(n-1)]) 

    def aire(self):
        res = 0.0
        for i in range(self.n-2):
            a = Triangle(self.sommets[0], self.sommets[i+1], self.sommets[i+2]).aire()
            res += a
        return res

    def barycentre(self, nom=None):
        gres = Point(0,0)
        s = 0
        pts = []
        aires = []
        for i in range(self.n-2):    
            tr = Triangle(self.sommets[0], self.sommets[i+1], self.sommets[i+2])
            a  = tr.aire()
            g  = tr.barycentre()
            s += a
            gres =  gres + g*a
        gres = gres/s
        gres.nom = nom
        return gres
            
class Triangle(Polygone): 
    def __init__(self, A, B, C):
        self.n = 3
        self.sommets=[A, B, C]
 
    def __repr__(self):
        return 'Triangle' + str(self.sommets)

    def aire(self):
        u = vecteur(self.sommets[0],self.sommets[1])
        v = vecteur(self.sommets[0],self.sommets[2])
        return abs(Vecteur.det(u,v))/2

    def barycentre(self,nom=None):
        u = self.sommets[0]
        v = self.sommets[1]
        w = self.sommets[2]
        return Point((u.x + v.x + w.x)/3, (u.y + v.y + w.y)/3,nom)

    def mediane(self, i):
        return Segment(self.sommet(i), (self.sommet(i+1) + self.sommet(i+2))/2)

    def hauteur(self, i, limited= False):
        if limited:
            return Segment(self.sommet(i),projection(self.sommet(i), (droite(self.sommet(i+1),self.sommet(i+2)))))
        else:
            return (droite(self.sommet(i+1),self.sommet(i+2))).orthogonale(self.sommet(i))
            
    def mediatrice(self,i):
        A, B = self.sommet(i+1),self.sommet(i+2)
        return orthogonale(droite(A,B),(A+B)/2)

    def barycentre(self, nom = None):
        G  = (self.sommet(0) + self.sommet(1) + self.sommet(2))/3
        G.nom = nom
        return G

    def orthocentre(self, nom = None):
        return self.hauteur(0).intersection(self.hauteur(1),nom=nom)

    def centreCercleCirconscrit(self,nom = None):
        return self.mediatrice(0).intersection(self.mediatrice(1),nom)

    def rayonCercleCirconscrit(self):
        return self.centreCercleCirconscrit().distance(self.sommet(0))

    def cercleCirconscrit(self):
        return Cercle(self.centreCercleCirconscrit(),self.rayonCercleCirconscrit())

    def piedHauteur(self,i,nom=None):
        return projection(self.sommet(i),droite(self.sommet(i+1),self.sommet(i+2)),nom)
        
        

        
        
        
    