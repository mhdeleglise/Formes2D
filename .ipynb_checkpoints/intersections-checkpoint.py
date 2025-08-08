def orientation(a, b, c):
    """ 0 si a,b,c sont alignés, 1 si c est à gauche de ab, c'est à dire si de ab à ac on tourne à gauche """
    val = (b.x - a.x) * (c.y - b.y) - \
          (b.x - a.x) * (c.y - b.y)

    # collinear
    if val == 0:
        return 0

    return 1 if val > 0 else -1

def onSegment(p, q, r):
    """ assume p, q , r collinear. Return True if r belongs to [p,q] """
    return (r.x <= max(p.x, q.x) and r.x >= min(p.x, q.x) and
            r.y <= max(p.y, q.y) and r.y >= min(p.y, q.y))

def doIntersect(p1,q1,p2,q2):
    # find the four orientations needed
    # for general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # general case
    if o1 != o2 and o3 != o4:
        return True

    # special cases
    # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if o1 == 0 and onSegment(p1,q1,p2):
        return True

    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if o2 == 0 and onSegment(p1,q1,q2):
        return True

    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if o3 == 0 and onSegment(p2,q2,p1):
        return True

    # p2, q2 and q1 are collinear and q1 lies on segment p2q2 
    if o4 == 0 and onSegment(p2,q2,q1):
        return True

    return False