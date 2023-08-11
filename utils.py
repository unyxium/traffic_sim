# Simple utilities
# NOTE: does not handle curve objects, only primitives

import math

def normalise_vec(x, y):
    """Normalise a vector."""
    len = math.sqrt(x*x + y*y)
    return (x/len, y/len)


def offset_pt(point, tangent, distance=1):
    """Offset a point based on a tangent and offset distance. Offset occurs perpendicularly to the right of the vector. Tangent is not normalised."""
    return (point[0]+tangent[1]*distance, point[1]-tangent[0]*distance)


def move_pt(point, tangent, distance=1):
    """Move a point along a tangent. Tangent is not normalised."""
    return (point[0]+tangent[0]*distance, point[1]+tangent[1]*distance)


def vec_bearing(vector):
    """Get the direction of a 2D vector. Angle is a bearing starting from +y and going clockwise."""
    return 90-math.degrees(math.atan2(vector[1],vector[0]))


def angle_difference(angle1, angle2):
    """Get the difference of 2 angles, positive if angle2 is CW from angle1. Wraps at +/-180."""
    return ((angle2-angle1)+180) % 360 - 180


def reverse_vec(vector):
    """Reverse a vector."""
    return (0-vector[0], 0-vector[1])


def scale_vec(vector, scale=1):
    """Scale a vector."""
    return (scale*vector[0], scale*vector[1])


def switch(control, val_true, val_false):
    """Stream filter, return val_true or val_false dependent on control boolean."""
    return val_true if control else val_false


def kph(speed):
    """Convert km/h to m/s and round result."""
    return round(speed*0.277778, 3)


def vec_2pts(pt0, pt1):
    """Get a vector from 2 points."""
    return(pt1[0]-pt0[0], pt1[1]-pt0[1])


def const_accel(speed_start, speed_end, distance):
    """Calculate the constant acceleration needed to reach a target speed and distance, from a current speed."""
    # https://lambdageeks.com/how-to-find-acceleration-with-velocity-and-distance/

    if distance == 0:
        return 0
    return (speed_end**2 - speed_start**2)/(2*distance) # seems to be correct


def remap(value, value_domain=(0,1), output_domain=(0,1), clamp=False):
    """Map a value from an input range into an output range. 
    Domains may be a tuple of (min,max) or a single number signifying the maxima (0,max).\n
    Clamp will ensure no values go beyond the output domain and any that could are set to the domain limit."""

    if not isinstance(value_domain, tuple):
        value_domain = (0, value_domain)
    if not isinstance(output_domain, tuple):
        output_domain = (0, output_domain)

    if clamp:
        value = max(min(value, value_domain[1]), value_domain[0])
    
    if value_domain[1] - value_domain[0] == 0:
        return output_domain[0] # prevent division by 0 when mapping from a domain range of 0
    
    return output_domain[0] + (output_domain[1]-output_domain[0]) * ((value-value_domain[0])/(value_domain[1]-value_domain[0]))


def distribution_map(input_data, output_data, mode='weave', left_bias=True):
    """Generate a map from input to output lists. Return dict with list contents mapped.\n
    Note that rounding means bias can occur, which matters in left vs right driving roads. Set the bias parameter to fix this."""

    #print(input_data, output_data)

    mapping = {x:[] for x in input_data} # create empty dict with keys and lists
    
    if input_data and output_data: # ensure both lists contain something
        if mode == 'all':
            # map everything to everything
            for input_i, input_elem in enumerate(input_data): 
                for output_elem in output_data:
                    mapping[input_elem].append(output_elem)
        
        elif mode == 'weave':
            
            for input_i, input_elem in enumerate(input_data): 
                # map index is a float between 0 and last exit lane index indicating where exactly it maps to. 
                # it gets rounded later to find the closest output.
                map_index = remap(input_i, (-0.5, len(input_data)-0.5), len(output_data)-1, True)
                #mapping[input_elem].append(map_index)
                if output_data[math.floor(map_index)] not in mapping[input_elem]:
                    mapping[input_elem].append(output_data[math.floor(map_index)])
                
                if output_data[math.ceil(map_index)] not in mapping[input_elem]:
                    mapping[input_elem].append(output_data[math.ceil(map_index)])

                #mapping[input_elem].append(output_data[round(map_index)])

            #for output_i in range(output_data):


    
    return mapping


class CyclicPair:
    """Iterator for cyclic adjacent pairs of integers. Returns indices only!"""
    def __init__(self, count):
        self.count = count

    def __iter__(self):
        self.i = 0
        return self

    def __next__(self):
        if self.i < self.count:
            pair = (self.i, (self.i+1)%(self.count))
            self.i += 1
            return pair
        else:
            raise StopIteration


class Pair:
    """Iterator for adjacent pairs from an iterable."""

    def __init__(self, iterable):
        self.iterable = iterable
        self.count = len(iterable)-1

    def __iter__(self):
        self.i = 0
        return self

    def __next__(self):
        if self.i < self.count:
            pair = (
                self.iterable[self.i], 
                self.iterable[self.i+1]
                )
            self.i += 1
            return pair
        else:
            raise StopIteration



if __name__ == '__main__':
    #for a in CyclicPair(5):
    #   print(a)
    """print(offset((0, 0), (2, 2), 1))
    print(offset((0, 0), (-1, 2), 1))
    print(offset((0, 0), (-4, -4), 1))
    print(offset((0, 0), (3, -4), 1))

    print('bearing tests')
    print(vec_bearing((1,0)))
    print(vec_bearing((1,1)))
    print(vec_bearing((0,1)))
    print(vec_bearing((-1,1)))
    print(vec_bearing((-1,0)))
    print(vec_bearing((-1,-1)))
    print(vec_bearing((0,-1)))
    print(vec_bearing((1,-1)))"""

    #print(list(CyclicPair(4)))
    #print(list(Pair('abcdef')))

    """print(angle_difference(0,100))
    print(angle_difference(50,100))
    print(angle_difference(-50,100))
    print(angle_difference(0,190))
    print(angle_difference(40,190))
    print(angle_difference(160,-160))
    print(angle_difference(180,-180))"""

    #print(remap(4, (2,3), (10,20), False))

    """print(distribution_map('a','1234'))
    print(distribution_map('ab','1234'))
    print(distribution_map('abc','1234'))
    print(distribution_map('abcd','123'))
    print(distribution_map('abcd','12'))
    print(distribution_map('abcd','1'))"""


