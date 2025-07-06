from Geo2d import *

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
    """ La perpendiculaire à self passant par P """
    a, b, x, y = d.a, d.b, P.x, P.y   
    return Droite(-b, a, b*x - a*y)    


   
    