import math
from utils import *


def offset_pt_2D(o, tangent, x, y):
    """Offset a point in 2D."""
    pt = (o[0] + x*tangent[1] + y*tangent[0], o[1] - x*tangent[0] + y*tangent[1])
    return pt


def dist_pt_to_line_segment(pt, l1, l2):
    """Find the distance from a point to a line segment."""

    length = math.dist(l1, l2)
    if length == 0: # degenerate line segment = point
        return math.dist(pt, l1)
    
    h = ((l2[0]-l1[0])*(pt[0]-l1[0]) + ((pt[1]-l1[1])*(l2[1]-l1[1])))/(length**2)
    if h < 0:
        return math.dist(pt, l1)
    elif h > 1:
        return math.dist(pt, l2)
    else:
        return abs((l2[0]-l1[0])*(l1[1]-pt[1]) - ((l1[0]-pt[0])*(l2[1]-l1[1])))/length