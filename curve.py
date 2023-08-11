# GENERIC CURVES
# IF IMPLEMENTED IN GRASSHOPPER THIS WHOLE FILE IS REPLACED
# node order / params must match the network representation
# the functions here need to be replicated for all curve types, even GH.

import math
from utils import *

class Line:
    """A line segment defined by 2 end points."""

    def __init__(self, n1, n2):
        assert isinstance(n1, (tuple, list)), 'n1 is not a coordinate'
        assert isinstance(n2, (tuple, list)), 'n1 is not a coordinate'
        assert n1 != n2, 'line has 0 length'
        
        self.n1 = n1 # node 1 coords
        self.n2 = n2 # node 2 coords

    def __repr__(self):
        return f'Line({self.n1},{self.n2})'

    def data(self):
        return {'crv_type':'Line','n1':self.n1,'n2':self.n2}

    def length(self):
        """Total length of the curve."""
        return math.dist(self.n1, self.n2)

    def eval(self, t):
        """Get the position and tangent given a normalised parameter value."""
        t = min(max(0, t), 1)  # clamp value

        px = self.n1[0] + t*(self.n2[0]-self.n1[0])  # pos
        py = self.n1[1] + t*(self.n2[1]-self.n1[1])

        tx = (self.n2[0]-self.n1[0])/math.dist(self.n1, self.n2)  # tangent, normalised
        ty = (self.n2[1]-self.n1[1])/math.dist(self.n1, self.n2)

        return (px, py), (tx, ty)

    def curvature(self, t):
        """Get the curvature"""
        return 0

class QuadraticBezier:
    """A quadratic bezier curve."""

    LIMIT = 0.1 # limit before it 

    def __init__(self, n1, cp, n2):
        assert isinstance(n1, (tuple, list)), 'n1 is not a coordinate'
        assert isinstance(cp, (tuple, list)), 'cp is not a coordinate'
        assert isinstance(n2, (tuple, list)), 'n1 is not a coordinate'
        
        self.n1 = n1 # node 1 coords
        self.cp = cp # control point coords
        self.n2 = n2 # node 2 coords

        # length, cached
        last_pt, _ = self.eval(0)
        self._total_length = 0
        for t in range(10):
            pt, _ = self.eval(t+1/10)
            self._total_length += math.dist(last_pt, pt)
            last_pt = pt
        

    def __repr__(self):
        return f'QBezier({self.n1},{self.cp},{self.n2})'

    def data(self):
        return {'crv_type':'QBezier','n1':self.n1,'cp':self.cp,'n2':self.n2}
    
    def length(self):
        """Total length of the curve."""
        return self._total_length

    def eval(self, t):
        """Get the position and tangent given a normalised parameter value."""
        t = min(max(0, t), 1)  # clamp value

        # pos
        px = (1-t)*((1-t)*self.n1[0] + t*self.cp[0]) + \
            t*((1-t)*self.cp[0] + t*self.n2[0])
        py = (1-t)*((1-t)*self.n1[1] + t*self.cp[1]) + \
            t*((1-t)*self.cp[1] + t*self.n2[1])
        
        #print('info', t, self.n1, self.cp, self.n2)
        # https://stackoverflow.com/questions/38499211/get-normal-of-bezier-curve-in-2d
        tx = self.n1[0]*(2*t-2) + \
            (2*self.n2[0] - 4*self.cp[0])*t + \
            2*self.cp[0]
        ty = self.n1[1]*(2*t-2) + \
            (2*self.n2[1] - 4*self.cp[1])*t + \
            2*self.cp[1]

        return (px, py), normalise_vec(tx, ty)
    
    def curvature(self, t):
        """Find the curvature (kappa) at a given normalised parameter."""
        # https://math.stackexchange.com/questions/220900/bezier-curvature
        # https://en.wikipedia.org/wiki/B%C3%A9zier_curve
        # https://pomax.github.io/bezierinfo/#curvature
        # find unnormalised vel and accel

        vx = self.n1[0]*(2*t-2) + \
            (2*self.n2[0] - 4*self.cp[0])*t + \
            2*self.cp[0]
        vy = self.n1[1]*(2*t-2) + \
            (2*self.n2[1] - 4*self.cp[1])*t + \
            2*self.cp[1]

        ax = 2*(self.n2[0] - 2*self.cp[0] + self.n1[0])
        ay = 2*(self.n2[1] - 2*self.cp[1] + self.n1[1])

        determinant = vx * ax - vy * ay
        speed = math.sqrt(vx**2 + vy**2)

        return abs(determinant) / speed**3


class PolyLine:
    """A polyline curve defined by any number of points."""
    
    def __init__(self, points):
        # should be a list of tuples
        if len(points) < 2:
            raise Exception('too few points')
        
        assert isinstance(points, list)
        self.points = points

        # cache
        total_length = 0
        cumulative_lengths = [0] # each length, to be divided to get t
        
        for i in range(len(self.points)-1):
            total_length += math.dist(self.points[i], self.points[i+1])
            cumulative_lengths.append(total_length)

        self._total_length = total_length
        self.t_values = [l/total_length for l in cumulative_lengths] # the t values for each point (inc. start at t=0 and end at t=1)

        # used for curve extension so it can be more accurate. Not for use in GH. Does not get set automatically.
        self.custom_tangent_start = None
        self.custom_tangent_end = None

        # TODO cache for curvature
    
    def __repr__(self):
        return f'PolyLine({self.points})'

    def data(self):
        return {'crv_type':'PolyLine','pts':self.points}
    
    def length(self):
        """Total length of the curve.""" 
        return self._total_length

    def eval(self, t):
        """Get the position and tangent given a normalised parameter value."""

        # t at a kink will favour the start of the next segment

        if t >= 1:
            n1 = self.points[-2]
            n2 = self.points[-1]
            
            tx = (n2[0]-n1[0])/math.dist(n1, n2)  # tangent, normalised
            ty = (n2[1]-n1[1])/math.dist(n1, n2)
            
            return n2, (tx, ty)

        for i, val in enumerate(self.t_values):
            if val > t: # check if val is bigger than t and stop if it is
                
                # pts
                n1 = self.points[i-1]
                n2 = self.points[i]
                
                t = (t - self.t_values[i-1])/(self.t_values[i] - self.t_values[i-1]) # unlerp t

                px = n1[0] + t*(n2[0]-n1[0])  # pos
                py = n1[1] + t*(n2[1]-n1[1])

                tx = (n2[0]-n1[0])/math.dist(n1, n2)  # tangent, normalised
                ty = (n2[1]-n1[1])/math.dist(n1, n2)

                return (px, py), (tx, ty)


        raise Exception('could not find t', t)

    def curvature(self, t):
        """Linear interpolation of curvature."""

        return 0

class Circle:
    """A circle."""

    def __init__(self, center_pt, radius=1):
        self.center_pt = center_pt
        self.radius = radius

    def __repr__(self):
        return f'Circle({self.center_pt}, r{self.radius})'
    
    def data(self): # this class shouldn't be used as a curve, it's only for geo intersections
        return {'crv_type':'Circle','center_pt':self.center_pt,'radius':self.radius} 
        
    



###### CURVE GENERATORS

def offset_curve(curve, offset, steps=10):
    """Offset a curve and return the offset."""
    # generates a polyline only due to limitations, GH won't

    points = []
    for t in range(steps+1):
        pt = curve.eval(t/steps)
        points.append(offset_pt(pt[0], pt[1], offset))

    # can't just return the polyline, must change the t values to align with bezier t
    output_curve = PolyLine(points)
    output_curve.t_values = [t/steps for t in range(steps+1)]

    pt = curve.eval(0)
    output_curve.custom_tangent_start = (offset_pt(pt[0], pt[1], offset), pt[1])
    pt = curve.eval(1)
    output_curve.custom_tangent_end = (offset_pt(pt[0], pt[1], offset), pt[1])
    return output_curve


def line_SDL(start_pt, tangent, length=1):
    """Create a line segment defined by start point, tangent and length."""
    return Line(start_pt, (start_pt[0]+tangent[0]*length, start_pt[1]+tangent[1]*length))


def extend_curve(curve, extension_start, extension_end):
    """Extend a curve on both ends linearly."""

    if isinstance(curve, PolyLine):
        new_points = list(curve.points)
        if extension_start != 0:
            new_points.insert(0, move_pt(curve.custom_tangent_start[0], curve.custom_tangent_start[1], -extension_start))
        if extension_end != 0:
            new_points.append(move_pt(curve.custom_tangent_end[0], curve.custom_tangent_end[1], extension_end))
        
        #print(len(new_points))
        return PolyLine(new_points)
    
    return NotImplemented


###### GEO INTERSECTION

def line_line_intersection(p1, p2, p3, p4):
    """Utility to find the intersection of 2 line segments."""
    denominator = (p1[0]-p2[0])*(p3[1]-p4[1]) - (p1[1]-p2[1])*(p3[0]-p4[0])
    if denominator == 0:
        return None
    
    t = ((p1[0]-p3[0])*(p3[1]-p4[1])-(p1[1]-p3[1])*(p3[0]-p4[0])) / denominator
    u = ((p1[0]-p3[0])*(p1[1]-p2[1])-(p1[1]-p3[1])*(p1[0]-p2[0])) / denominator

    if t >= 0 and t <= 1 and u >= 0 and u <= 1:
       return (p1[0]+t*(p2[0]-p1[0]), p1[1]+t*(p2[1]-p1[1]))
    
    return None


def line_circle_intersection(line_p1, line_p2, circle_center, cicle_radius):
    """Utility to find the intersections of a line segment and a circle."""

    a = math.dist(line_p1, line_p2)**2
    b = 2 * ((line_p1[0]-circle_center[0])*(line_p2[0]-line_p1[0]) + (line_p1[1]-circle_center[1])*(line_p2[1]-line_p1[1]))
    delta = (b**2) - (4*a * ((math.dist(line_p1, circle_center)**2) - (cicle_radius**2)))

    if delta < 0:
        return None
    
    t1 = (-b - math.sqrt(delta)) / (2*a)
    if t1 > 1 or t1 < 0:
        t1 = None
    
    t2 = (-b + math.sqrt(delta)) / (2*a)
    if t2 > 1 or t2 < 0:
        t2 = None

    return (t1, t2)


def polyline_polyline_intersection(polyline1_points, polyline2_points):
    """Utility to find the intersection of 2 polylines, given lists of control points. \n
    DOES NOT USE THE CLASS IMPLEMENTATION OF POLYLINE"""

    for pl1_a, pl1_b in Pair(polyline1_points):
        for pl2_a, pl2_b in Pair(polyline2_points):
            intersection = line_line_intersection(pl1_a, pl1_b, pl2_a, pl2_b)
            if intersection is not None:
                return intersection
    return None


def polyline_circle_intersection(polyline_points, circle_center, circle_radius):
    """Utility to find the intersection of a polyline (list of points) and a curve (pt and radius). \n
    DOES NOT USE THE CLASS IMPLEMENTATION OF POLYLINE"""

    all_intersections = []

    for pl_a, pl_b in Pair(polyline_points):
        intersection = line_circle_intersection(pl_a, pl_b, circle_center, circle_radius)
        if intersection is not None:
            if intersection[0] is not None:
                all_intersections.append(intersection[0])

    return None


def curve_curve_intersection(curve1, curve2):
    """Find the intersection point of two generic curves."""

    if isinstance(curve1, PolyLine) and isinstance(curve2, PolyLine):
        return polyline_polyline_intersection(curve1.points, curve2.points)
    
    elif isinstance(curve1, Line) and isinstance(curve2, Line):
        return line_line_intersection(curve1.n1, curve1.n2, curve2.n1, curve2.n2)
    
    elif isinstance(curve1, PolyLine) and isinstance(curve2, Circle):
        #return polyline_circle_intersection(curve1.points, curve2.center_pt, curve2.radius)
        all_intersections = []

        i = 0
        for pl_a, pl_b in Pair(curve1.points):
            intersection = line_circle_intersection(pl_a, pl_b, curve2.center_pt, curve2.radius)
            if intersection is not None:
                if intersection[0] is not None:
                    # lerp between t+0 and t+1 with intersection
                    all_intersections.append(curve1.t_values[i] + intersection[0] * (curve1.t_values[i+1] - curve1.t_values[i]))
                
                if intersection[1] is not None:
                    # lerp between t+0 and t+1 with intersection
                    all_intersections.append(curve1.t_values[i] + intersection[1] * (curve1.t_values[i+1] - curve1.t_values[i]))

            i += 1
        
        return sorted(all_intersections)
    
    elif isinstance(curve1, Circle) and isinstance(curve2, PolyLine):
        raise Exception('polyline should be ordered first')
    
    print('unrecognised curve types')
    return None


if __name__ == '__main__':
    print('test')
    # tests
    """l = Line((12, 13), (16, -5))
    print(l.__class__.__name__)
    print(l)
    print(l.length())
    print(l.eval(0.1))"""

    #b = QuadraticBezier((0, 0), (5, 0), (33, 10))
    b = QuadraticBezier((0,0), (5,0), (5,-5))
    # seems right
    #print(b.__class__.__name__)
    #print(b.eval(0.5))
    #print(b.length())
    #print('')
    c = QuadraticBezier((0,0), (50,0), (50,50))
    print(c.curvature(0))
    print(c.curvature(0.25))
    print(c.curvature(0.5))
    print(c.curvature(0.75))
    print(c.curvature(1))
    print(c.eval(0))
    print(c.eval(0.25))
    print(c.eval(0.5))
    print(c.eval(0.75))
    print(c.eval(1))
    #print(c.eval(0))
    #print(c.eval(1))

    pl = PolyLine([(0,0), (5,0), (5,-5), (20,-5)])
    #print(pl.length())
    #print(pl.eval(0))
    #print(pl.eval(1))
    #print(pl.eval(0.01))
    #print(pl.eval(0.2))
    """print('>>>')

    b2 = offset_curve(b, 10, 5)
    print(len(b2.points))
    print(b2.points)

    ext1 = extend_curve(b2, 20, 20)
    
    print('>>>')
    print(len(ext1.points))
    print(ext1.points)
    print('')
    print(len(ext1.t_values))
    print(ext1.t_values)
    print('>>>')

    pl = PolyLine([(216.0, 140.0), (236.0, 137.0), (237.85, 137.0), (239.7, 137.0), (241.55, 137.0), (243.4, 137.0), (245.25, 137.0), (247.1, 137.0), (248.95, 137.0), (250.8, 137.0), (252.65, 137.0), (254.5, 137.0), (256.35, 137.0), (258.2, 137.0), (260.05, 137.0), (261.9, 137.0), (263.75, 137.0), (265.6, 137.0), (267.45, 137.0), (269.3, 137.0), (271.15, 137.0), (273.0, 137.0), (293.0, 140.0)])
    circ = Circle((272.49537413695043, 137.0), 3)

    print(curve_curve_intersection(pl, circ))"""
    #print(polyline_polyline_intersection([(2,2), (35,35)], [(1,4), (5,0)]))
    
    #assert line_line_intersection((2,2), (35,35), (1,4), (5,0)) == (2.5, 2.5)
    #print('lci')
    #print(line_circle_intersection((1,1), (5,5), (0,0), 1.4))

    #print(curve_curve_intersection(PolyLine([(0,0),(5,0),(10,5),(5,5)]), Circle((3,3), 4)))