def orientation(a, b, c):
    """ Renvoie 0 si abc sont alignés, 1 si on tourne à gauche en b et -1 si on tourne à droite """
    val = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)
          
    if val == 0:
        return 0

    return 1 if val > 0 else -1

def onSegment(a , b, c):
    """ a,b,c sont alignés. renvoie True if c appartient au segment ab """
    return (c.x <= max(a.x, b.x) and c.x >= min(a.x, b.x) and
            c.y <= max(a.y, b.y) and c.y >= min(a.y, b.y))

def doIntersect(p1,q1,p2,q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # non aligneés
    if o1 != o2 and o3 != o4:
        return True

    # p2 entre p1 et q1
    if o1 == 0 and onSegment(p1,q1,p2):
        return True

    # q2 entre p1 et q1
    if o2 == 0 and onSegment(p1,q1,q2):
        return True

    # p1 entre p2 et q2
    if o3 == 0 and onSegment(p2,q2,p1):
        return True

    # q1 entre p2 et q2
    if o4 == 0 and onSegment(p2,q2,q1):
        return True

    return False