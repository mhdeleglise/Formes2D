import Geo
from Geo import *
from Polygones import *
from math import pi as pi, sin as sin, tan as tan, cos as cos, radians as radians, degrees as degrees

q0 = Polygone([Point(-1/2,1/2), Point(1/2,1/2), Point(1/2,-1/2),  Point(-1/2,-1/2)])
O  = Point(0,0,'O')

class Regle(Polygone):
    def __init__(self,theta,  h):
        super().__init__(q0.rotation(theta).sommets)
        self.theta = theta
        self.h   = h
        if theta == 0:
            self.sommets= list(q0.sommets)
            self.E  = Point(-1/2,h,'E')
            self.F  = Point(1/2,h, 'F')
            self.qi = Polygone([self.E, self.F, q0.sommets[2], q0.sommets[3]])
        else:
            qt = q0.rotation(theta)
            self.A, self.B, self.C, self.D = qt.sommets[0], qt.sommets[1], qt.sommets[2], qt.sommets[3]
            if h > self.A.y:
                self.E  = Point(self.A.x + (h-self.A.y)/tan(theta),h,'E')
                self.F  = Point(self.B.x + (self.B.y-h)*tan(theta),h,'F')
                self.qi = Polygone([self.E,self.F,self.C,self.D,self.A])
            else:
                self.E  = Point(self.A.x + (self.A.y-h)*tan(theta),h,'E') 
                self.F  = Point(self.B.x + (self.B.y-h)*tan(theta),h,'F')
                self.qi = Polygone([self.E,self.F,self.C,self.D])
        self.P = Point(self.qi.isobarycentre().x, self.qi.isobarycentre().y,'P')
        self.densite = self.qi.aire()
        self.equilibre = abs(self.P.x) < 1e-9
        self.rM = Point.distance(self.E,self.F)**3/12/self.qi.aire() # Le rayon métacentrique
        self.M  = Point(0,self.rM + self.P.y,'M') # Le métacentre
        self.H  = Point(0,self.h,'H')
        self.stable = self.M.y >= 0        

    def __repr__(self):
        return 'Regle' + str(self.sommets) + 'd = ' + str(self.densite) + '   h = ' + str(self.h)
        
    def draw(self, axes= 'off', labels = False, center = True, color = "yellow", save = False):        
        fig, ax = plt.subplots(layout= 'constrained')
        fig.set_size_inches(8,5)        
        xm = 1.10
        ymin, ymax = -xm, xm
        xmin, xmax = -xm, xm     
        plt.ylim(ymin, ymax)
        plt.xlim(xmin, xmax)
        super().draw(color=color)
        title = 'Équilbre stable, densité {:8.6f}, $\\theta$= {:8.6f}°'.format(self.densite,degrees(self.theta))
        if self.P.x < -1e-8:
            title = 'Déséquilibre, pivote vers la droite'
        elif self.P.x > 1e-8:
            title = 'Déséquilibre, pivote vers la gauche'   
        elif self.M.y < 1e-9:
            title = 'Équilibre instable'
        fig.suptitle(title,color="green", fontsize = 16)
        self.E.draw(ax, dx= -0.06, dy=-0.01, s=10, color='red', namecolor='blue')
        self.F.draw(ax, dx= -0.06, dy=-0.01, s=10, color='red', namecolor='blue')
        self.P.draw(ax, dx= -0.06, dy=-0.01, s=10, color='red', namecolor='red')
        self.H.draw(ax, dx= -0.06, dy=-0.01, s=8, color='blue', namecolor='blue')        
        if abs(self.P.x) < 1e-9:
            self.M.draw(ax, dx= -0.06, dy=-0.01, s=8, color='red', namecolor='red')
        O.draw(ax, dx=0.04,        dy=-0.01, color='blue', namecolor='blue',s=8)
        plt.axhspan(ymin, self.h, color='blue', alpha=0.1)
        line1  = 'Densité={:12.9f}             $\\theta$= {:8.6f}°            OH= {:10.9f}'.format(self.densite, degrees(self.theta), self.h)  
        if self.equilibre:
            line2 = 'Ordonnée de M:  {:10.9f}'.format(self.M.y)
        else:
            line2 = 'Abscisse de P:  {:10.9f}'.format(self.P.x)
        ax.text( -1.20, ymin+0.01, line1,color='blue',fontsize=11)
        ax.text( -0.46, ymin + 0.15, line2, color='blue',fontsize=11)
        plt.axis('equal')
        ax.axis(axes)
        if save == True:
            imagename = 'regle_{:6.3f}_{:7.6f}.png'.format(degrees(self.theta), self.h)
            plt.savefig(imagename)
        
def rotation(self, theta):
    O1 = (self.E+self.F)/2
    print("E,F,O1= ", E,FO1)
    return super().rotation(theta,O1)

def dichoto(theta, a, b):
    while (b-a)> 1e-12:
        c = (a+b)/2
        qu, qw = Regle(theta,a), Regle(theta,c)
        if qu.P.x * qw.P.x > 0:
            a = c
        else:
            b = c
    return (a+b)/2

def find(theta):
    for n in range(100):
        qu, qv = Regle(theta,n/100), Regle(theta,(n+1)/100)
        if qu.P.x * qv.P.x < 0:
            return n
    return 0

def solution(theta, color = 'yellow', fname='nosave'):
    if abs(theta) < 1e-8:
        print('Une solution horizontale pour d dans [0.788679,1')
        return
    if abs(theta-pi/4) < 1e-8:
        print("Une solution d'inclinaison 45° pour d dans [1/2, 23/32] = [0.5, 0.71875]")
        return
    n = find(theta)
    h = dichoto(theta,n/100,(n+1)/100)
    return Regle(theta, h)

