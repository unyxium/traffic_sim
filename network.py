# Handles the network - roads, lanes and nodes.

from typing import List, Literal
from curve import Line, QuadraticBezier, PolyLine, Circle, offset_curve, extend_curve, curve_curve_intersection
from utils import *
import random
import math

sort_by_bearing = lambda dictionary: dictionary['bearing']
debug_geometry = []

CRV_EVAL_BUFFER = 0 # just in case the ends are degenerate, set >0
PATHFIND_DEPTH_LIMIT = 10000 # pathfinding only
VEHICLE_MIN_SPACING = 7 # metres between vehicles minimum
FWD_STOP_BUFFER = 4
TOL = 0.001 # bezier tolerance
ROUTE_TGT_DIST_LIMIT = 150 # how far can targets be searched from the start of the road

class Node:
    """A single node."""

    def __init__(self, x, y, z=0):
        # essentials
        self.x = x
        self.y = y
        self.z = z # optional, gives flexibility with visualisations if it can be stored

        # ins is short for intersection
        self.ins_type = None # none for no logic. traffic light, stop
        self.ins_lane_priorities = {} # matches lane_mapping dict, for vehicles to lookup and cache themselves.
        self.ins_tl_behaviour = None # traffic light behaviour (groups of lanes, durations, type of signal)
        
        # caches (automatically generated)
        self.connections = [] # cyclic list of road connection data, lanes can be found from this via road class, auto gen
        self.crv_intersections = [] # cyclic list of crv geo intersection points, following connections list, auto gen
        self.lane_mapping = {} # lookup for which road and lane goes where. dict of roads, dict of lanes per rd, list of roads and lanes to go to

        self.mapping_curves = {} # Key should be a tuple of (R,L,R,L) for entering and exiting

        self.all_lanes_approaching = [] # list of lanes, only used in branch cache and pygame viewer
        self.all_lanes_exiting = [] # list of lanes, only used in branch cache and pygame viewer

    def __repr__(self):
        if self.z == 0:
            return f'Node({self.x},{self.y})'
        return f'Node({self.x},{self.y},{self.z})' # 3D version

    def data(self):
        """Return data as a dictionary."""
        return {
            'x':self.x,
            'y':self.y,
            'z':self.z,
            'ins_type':self.ins_type,
            'ins_tl_behaviour':self.ins_tl_behaviour,
            'connections':self.connections,
            'crv_intersections':self.crv_intersections,
            'lane_mapping':self.lane_mapping,
            'ins_lane_priorities':self.ins_lane_priorities,
            }
    
    def pos(self):
        """Return the node's position as a tuple"""
        return (self.x, self.y)
    
    def get_lane_mapping(self, road_entering, lane_index):
        """Get the lane mapping (lanes that can be visited entering an intersection).\n
        Returns a list of lanes, these lanes are dictionaries containing road & lane index, 
        away, fwd, and bearing."""
        if road_entering in self.lane_mapping:
            if lane_index in self.lane_mapping[road_entering]:
                return self.lane_mapping[road_entering][lane_index]
        print(f'no lane mapping found for R{road_entering}, L{lane_index}')
        return None

    def get_intersection_priority(self, road_entering, lane_index):
        """Get the intersection priority."""
        if road_entering in self.lane_mapping:
            if lane_index in self.lane_mapping[road_entering]:
                return self.ins_lane_priorities[road_entering][lane_index]
        print(f'no lane priority found for R{road_entering}, L{lane_index}')
        return None

class Lane:
    """A single road lane"""

    # the only type that matters is carriageway which are navigable by the vehicles
    # a later implementation may consider traffic type
    LANE_TYPES = Literal['carriageway', 'median', 'footpath', 'bus_lane', 'cycle_lane', 'parking']

    def __init__(self, type:LANE_TYPES='carriageway', width=3, fwd=True, speed=14):
        self.type = type # the type of lane (affects vehicle type and display).
        self.width = width  # width of lane
        self.fwd = fwd  # for the direction of lane types that can be travelled along
        self.speed = speed  # speed limit

    def __repr__(self):
        return f'Lane(type:{self.type}, width:{self.width}, fwd:{self.fwd})'
    
    def data(self):
        """Return data as a dictionary"""
        return {'type':self.type,'width':self.width,'fwd':self.fwd,'speed':self.speed}


ROAD_ALLEY = [
    Lane(width=2, fwd=False, speed=kph(30)), 
    Lane(width=2, fwd=True, speed=kph(30)), 
]

ROAD_STREET = [
    Lane(type='parking', width=2), 
    Lane(fwd=False), 
    Lane(fwd=True), 
    Lane(type='parking', width=2),
    ]

ROAD_DOUBLE_SEPARATED = [
    Lane(fwd=False, speed=kph(60)), 
    Lane(fwd=False, speed=kph(60)), 
    Lane(type='median', width=2.5), 
    Lane(fwd=True, speed=kph(60)), 
    Lane(fwd=True, speed=kph(60)), 
    ]

ROAD_GENERIC = [
    Lane(type='footpath', width=2), 
    Lane(fwd=False), 
    Lane(fwd=True), 
    Lane(type='footpath', width=2),
    ]

ROAD_ONE_WAY_1 = [
    Lane(width=3, fwd=True), 
]

ROAD_ONE_WAY_2 = [
    Lane(width=3, fwd=True), 
    Lane(width=3, fwd=True), 
]

ROAD_HIGHWAY = [
    Lane(width=3, fwd=False, speed=kph(100)), 
    Lane(width=3, fwd=True, speed=kph(100)), 
    ]

ROAD_BOULEVARD = [
    Lane(type='footpath', width=3.5),
    Lane(type='parking', width=2), 
    Lane(fwd=False, speed=kph(60)), 
    Lane(fwd=False, speed=kph(60)), 
    Lane(type='median', width=2.5), 
    Lane(fwd=True, speed=kph(60)), 
    Lane(fwd=True, speed=kph(60)), 
    Lane(type='parking', width=2),
    Lane(type='footpath', width=3.5),
    ]


class RoadSegment:
    """Simple class to store a segment. Note that there are no actual geometry calculations handled here, the curve object handles it."""
    # data is stored n1 to n2, left-to-right (LTR) always

    def __init__(self, n1, n2, curve, lanes=None, trim_start=(0,0), trim_end=(1,1), road_name=None):
        assert isinstance(n1, int)
        assert isinstance(n2, int)

        self.n1 = int(n1)  # index into nodes list
        self.n2 = int(n2)
        self.curve = curve # stores the curve the segment represents, can be anything. \
        # in grasshopper this could be actual curve geometry or an index to it \
        # in python this could be another class for a curve representation, e.g. bezier

        self.road_name = road_name # extra

        if lanes is None:  # default 2 lane road
            lanes = [Lane(fwd=False),Lane(fwd=True)]
        self.lanes: List[Lane] = lanes # Lanes must be a list
        
        self.n1_dir = curve.eval(0+CRV_EVAL_BUFFER)[1] # keep this 
        self.n2_dir = curve.eval(1-CRV_EVAL_BUFFER)[1] # buffer used just in case

        # the t values to shorten the ends of the roads, LTR
        self.trim_start = [trim_start[0], trim_start[1]] # trim values, lists for LTR [not tuples because of assignment]
        self.trim_end = [trim_end[0], trim_end[1]]
        
        # used for the rhino version, extend curves:
        self.trim_dist_calculate()

        self.analysis_occupancy_time_total = 0
        self.analysis_avg_speed = 0
        self.analysis_avg_speed_elements = 1

    def __repr__(self):
        return f'RoadSegment({self.n1},{self.n2})'

    def trim_dist_calculate(self):
        """Generate trim distances (measured in units), used in Rhino. All distances start from 0."""
        clen = self.curve.length()
        
        self.trim_dist_start = [self.trim_start[0]*clen, self.trim_start[1]*clen]
        self.trim_dist_end = [clen-self.trim_end[0]*clen, clen-self.trim_end[1]*clen]
        

    def data(self):
        """Return data as a dictionary."""
        d = {
            'n1':self.n1,
            'n2':self.n2,
            'curve':self.curve.data(),
            'lanes':[lane.data() for lane in self.lanes],
            'trim_start':self.trim_start,
            'trim_end':self.trim_end,
            'trim_dist_start':self.trim_dist_start,
            'trim_dist_end':self.trim_dist_end,
            'road_name':self.road_name,
            }
        
        return d
    
    def get_width(self):
        """Return the width of the road."""
        return sum([lane.width for lane in self.lanes])

    def get_x_offset(self, lane_index=0):
        """Get the x_offset of the lane center given a LTR lane index."""

        if lane_index < 0 or lane_index > len(self.lanes):
            raise IndexError('Lane index out of range')
        else:
            L_offset = sum([self.lanes[i].width for i in range(0, lane_index)])
            return L_offset + self.lanes[lane_index].width/2 - self.get_width()/2

    def get_trim(self, x_offset):
        """Get the start and end trim given an x_offset"""

        xt = x_offset/self.get_width() + 0.5  # unlerp 0-1
        t1 = xt * (self.trim_start[1] - self.trim_start[0]) + self.trim_start[0]
        t2 = xt * (self.trim_end[1] - self.trim_end[0]) + self.trim_end[0]
        return (t1, t2)

    def get_lane_end(self, lane_index, end=False):
        """Get lane end. Choose start or end point (using direction of road as reference). Return offset point and vector."""
        
        x_offset = self.get_x_offset(lane_index)
        
        if self.lanes[lane_index].fwd == end:
            # end
            eval = self.curve.eval(self.get_trim(x_offset)[1])
        else:
            # start
            eval = self.curve.eval(self.get_trim(x_offset)[0])
        
        if self.lanes[lane_index].fwd:
            return (offset_pt(eval[0], eval[1], x_offset), eval[1])
        return (offset_pt(eval[0], eval[1], x_offset), reverse_vec(eval[1]))


    def get_navigable_lanes(self, fwd=None):
        """Get the indices of the navigable lanes. Filter by direction using fwd bool or disable with None."""
        navigable = []
        for i, lane in enumerate(self.lanes):
            if fwd is None or lane.fwd == fwd:  # check if lane goes in the right direction
                if lane.type == 'carriageway':
                    navigable.append(i)
        return navigable



class Marker:
    """Data class for vehicle marker data. DOES NOT HANDLE LOGIC."""
    VEHICLE_TYPES = Literal['spawner_random', 'spawner_pathfind', 'destination']

    def __init__(self, road_index, lane_indices, t, type:VEHICLE_TYPES='spawner_random'): 
        # location
        self.road_index = road_index
        self.lane_indices = lane_indices
        self.t = t
        #print(lane_indices)
        # identity
        self.type = type # spawner_random, spawner_pathfind, destination
        self.name = None # to identify markers for pathfinding, must be unique

        # spawners only:
        self.spawn_spacing = 30 # seconds
        self.spawns_remaining = 1000 # decreases when a vehicle is spawned, stop when 0. None for infinite.
        self.timer = 0 # count up until next spawn to then reset to 0

        # route:
        self.route = [None for _ in lane_indices] # for each lane, gets copied to a vehicle

    def __repr__(self):
        return f'Marker({self.name} {self.type} R{self.road_index}L{self.lane_indices}T{self.t})'
    
    def data(self):
        """Get all marker data as a dictionary."""
        # TODO
        marker_data = {
            'road_index':self.road_index,
            'lane_indices':self.lane_indices,
            't':self.t,
            'type':self.type,
            'name':self.name,
            'spawn_spacing':self.spawn_spacing,
            'spawns_remaining':self.spawns_remaining,
            'route':self.route,
        }
        
        return marker_data


class Vehicle:
    """Data class for vehicle data. DOES NOT HANDLE LOGIC."""

    def __init__(self, road_index, lane_index, t): 
        # asterisk indicates generated once

        self.delete_flag = False

        # location includes the intersection a vehicle is entering from
        self.road_index = road_index  #* which road the vehicle follows
        self.lane_index = lane_index  #* LTR lane index, 0 is leftmost, direction is assumed (comes from lane)
        self.t = t  # 0-1 parameter on the curve

        self.path_following = 'road'  # road or node (for intersection)
        self.path = None #* store the path curve locally
        self.path_offset = None #* offset path value
        self.path_dir = None #* True if following in the same dir as path, False only if following actual roads going opp. dir. Not used in node paths.
        
        # cached road parameters, dependent on above RLT:
        self.path_length = None
        self.path_t_start = None
        self.path_t_end = None  #* where the current path ends
        self.road_speed_limit = None  #* units/sec, relies on speed limit and fwd dist
        
        # additional parameters, updated based on above:
        self.distance_along_path = None
        self.speed = 0  # units/sec
        self.t_speed = 0  # t/sec
        self.acceleration = 0

        # POSITION AND SPEED
        self.pos = (None, None) # visual position
        self.dir = (None, None) # visual direction

        self.fwd_dist = None  # units of distance clear in front
        self.tgt_speed_graph = [] # list of tuples for position in front and target speed
        
        # navigational data:
        self.behaviour = None  # random, pathfind (go to target)
        self.route = []  # List of (road, lane) to take. Roads are deleted the moment they are visited? Random traffic should use this too.
        self.prev_road_index = None
        self.prev_lane_index = None

        # ANIMATION
        self.animation = [] # stores vehicle location data
        

    def __repr__(self):
        return f'Vehicle(R{self.road_index} L{self.lane_index} T{self.t})'


    def initialise_data(self, road):
        """Set up essential properties based on current road."""

        self.path = road.curve
        self.path_offset = road.get_x_offset(self.lane_index)
        self.path_dir = road.lanes[self.lane_index].fwd
        self.pos = road.curve.eval(self.t)[0]
        self.dir = road.curve.eval(self.t)[1]
        self.path_length = road.curve.length()
        self.distance_along_path = switch(self.path_dir, self.t, 1-self.t) * self.path_length
        self.path_t_start = switch(
            self.path_dir, 
            road.get_trim(self.path_offset)[0], 
            road.get_trim(self.path_offset)[1],
            )
        self.path_t_end = switch(
            self.path_dir, 
            road.get_trim(self.path_offset)[1], 
            road.get_trim(self.path_offset)[0],
            )
        self.road_speed_limit = road.lanes[self.lane_index].speed
        
        if self.speed is None:
            self.speed = self.road_speed_limit


    def save_data_frame(self, time):
        """Get vehicle data relevant to a frame.\n
        Tuple data: x position, y position, bearing"""
        self.animation.append((round(time, 3), round(self.pos[0], 2), round(self.pos[1], 2), round(vec_bearing(self.dir), 2)))

    def animation_data(self):
        """Get vehicle animation data as a dictionary."""
        return {
            'pos':self.pos,
            'dir':self.dir,
            'behaviour':self.behaviour,
            'animation':self.animation,
        }

    def data(self):
        """Get all vehicle data as a dictionary."""

        vehicle_data = {
            'road_index':self.road_index,
            'lane_index':self.lane_index,
            't':self.t,
            'path_following':self.path_following,
            'behaviour':self.behaviour, 
            'route':self.route,
            'pos':self.pos,
            'dir':self.dir,
            'speed':self.speed,
        }

        if self.path_following == 'node':
            vehicle_data['path'] = self.path.data() # get curve geometry
        
        return vehicle_data


class RoadNetwork:
    """Road network consisting of nodes and road segments joining them."""

    def __init__(self):
        self.nodes: List[Node] = []
        self.roads: List[RoadSegment] = []
        self.vehicles: List[Vehicle] = []
        self.markers: List[Marker] = []

        self.properties = [] # a list of properties of the data, used to keep track of what has been modified or created
        self.vehicle_map = {} # cache for vehicle location, key is road index

        self.record = False
        self.time = 0 # the current time, will be incremented if record is true
        self.record_duration = 60 # how many seconds the recording will last


    def __str__(self):
        return f'<< Network with {len(self.nodes)} nodes and {len(self.roads)} roads >>'


    def add_node(self, x, y, z=0):
        """Create a node at a certain location."""

        for node in self.nodes:
            if node.pos() == (x, y, z):
                raise Exception(f'Node already exists at this location {(x,y,z)}')

        self.nodes.append(Node(x, y, z))


    def add_road(self, n1, n2, curve=None, lanes=None, trim_start=(0,0), trim_end=(1,1)):
        """Create a road segment between two nodes given their index."""

        if n1 >= len(self.nodes) or n2 >= len(self.nodes) or n1 < 0 or n2 < 0:
            raise Exception(f'Node index out of range, {len(self.nodes)} nodes currently exist')
        
        if curve is None: 
            if n1 == n2: # lines can't share the same node
                return
            new_curve = Line(self.nodes[n1].pos(), self.nodes[n2].pos()) # assume None means line
        elif isinstance(curve, tuple):
            new_curve = QuadraticBezier(self.nodes[n1].pos(), curve, self.nodes[n2].pos()) # assume tuple means Bezier
        else:
            new_curve = curve # assume the curve is a usable object

        self.roads.append(RoadSegment(n1, n2, new_curve, lanes, trim_start, trim_end))



    def add_spawner(self, road_index, lane_indices=[0], t=0.1, spacing=10, spawns_remaining=1000, target=None):
        """Add a new spawner marker. Target given, None for random."""
        
        if not isinstance(lane_indices, list):
            lane_indices = [lane_indices]
        
        m = Marker(road_index, lane_indices, t)
        
        # edit parameters
        m.spawn_spacing = spacing
        m.timer = m.spawn_spacing
        m.spawns_remaining = spawns_remaining
        
        if target is None or target == [None, None]:
            m.type = 'spawner_random'
        else:
            m.type = 'spawner_pathfind'
            # pathfinding needs to be per lane
            m.route = []
            for lane_index in lane_indices:
                #print(lane_index)
                m.route.append(self.create_vehicle_route((road_index, lane_index), target))
        
        self.markers.append(m)
        
        

    def add_vehicle(self, road_index, lane_index=0, t=0, route=None, speed=None):
        """Spawn a vehicle on a road segment. If no route is given, choose a random path."""

        v = Vehicle(road_index, lane_index, t) # the object is created and edited directly before being appended
        v.speed = speed
        if route is None:
            v.behaviour = 'random'
            #print('random route', (road_index, lane_index), v.route)
            v.route = self.create_vehicle_route((road_index, lane_index), target=None, depth_limit=3)
            
        else:
            #print('pathfind', route)
            v.behaviour = 'pathfind'
            v.route = route
            
        v.initialise_data(self.roads[v.road_index])
            
        self.vehicles.append(v)
    

    def add_random_traffic(self, count):
        for _ in range(count):
            for tries in range(50): # max retries per vehicle spawned
                road_index = random.randint(0, len(self.roads)-1)
                lanes = self.roads[road_index].lanes
                chosen_lane = random.randint(0, len(lanes)-1)
                trim = self.roads[road_index].get_trim(self.roads[road_index].get_x_offset(chosen_lane))

                if lanes[chosen_lane].type == 'carriageway':
                    self.add_vehicle(road_index, chosen_lane, remap(random.random(), output_domain=trim), speed=lanes[chosen_lane].speed*0.7)
                    break

            if tries >= 50:
                print('failed to add vehicles')
                break # retries for a vehicle was hit, just stop because it's likely no more can be added


    def recalculate_road(self, index):
        """Utility to recalculate road data to ensure accuracy. Run if the curve or nodes were changed."""

        road = self.roads[index]

        # update curve endpoints
        road.curve.n1 = self.nodes[road.n1].pos()
        road.curve.n2 = self.nodes[road.n2].pos()

        # update road dirs
        road.n1_dir = road.curve.eval(0+CRV_EVAL_BUFFER)[1] 
        road.n2_dir = road.curve.eval(1-CRV_EVAL_BUFFER)[1] 


    def generate_connection_cache(self):
        """Generate cyclic intersection cache. Indicates the roads that connect to each node. Necessary for other intersection operations."""
        if 'connection_cache' not in self.properties:
            self.properties.append('connection_cache')
        
        for i, node in enumerate(self.nodes):

            node_connections = [] # Connections hold static road data. Trim shouldn't be stored here, the road itself stores it. 
            
            for road_index, road in enumerate(self.roads): 
                if road.n1 == i:
                    node_connections.append({'road_index':road_index, 'away':True, 'bearing':vec_bearing(road.n1_dir), 'vec':road.n1_dir}) 
                if road.n2 == i:
                    node_connections.append({'road_index':road_index, 'away':False, 'bearing':vec_bearing(reverse_vec(road.n2_dir)), 'vec':reverse_vec(road.n2_dir)})
                    
            node_connections.sort(key=sort_by_bearing, reverse=True) # sort bearing, note list is reversed so it is CCW so right turns are processed

            node.connections = node_connections # add data to node {road index, fwd (T if away from node), dir away from node}


    def trim_roads(self, offset_crv_steps=20, curb_radius=1, extension=2):
        """Iterate over the cyclic cache and set the trim for the road. Requires connections cache."""
        if 'trim' not in self.properties:
            self.properties.append('trim')

        # Create a list of curve offsets, aligns with roads list.
        # store offsets in object, for debugging and later use. 
        self.oL = []
        self.oR = []
        for road in self.roads: # offset road curves and store in lists
            self.oL.append(offset_curve(road.curve, -0.5*road.get_width(), offset_crv_steps))
            self.oR.append(offset_curve(road.curve, 0.5*road.get_width(), offset_crv_steps))

        
        for node in self.nodes:

            node.crv_intersections = [] # A list of intersection points, starting from between i0 and i1, going CCW following node.connections

            for i0, i1 in CyclicPair(len(node.connections)): # must use index
                # connections: c0, c1
                c0 = node.connections[i0]
                c1 = node.connections[i1]

                # roads: r0, r1
                r0: RoadSegment = self.roads[c0['road_index']]
                r1: RoadSegment = self.roads[c1['road_index']]

                width0 = r0.get_width()
                width1 = r1.get_width()

                if c0 == c1: # both connections are the same = dead end road, special handling needed
                    node.crv_intersections.append(None)
                    continue

                # get correct side polyline offsets for both roads: pl0, pl1
                pl0 = switch(c0['away'], self.oL[c0['road_index']], self.oR[c0['road_index']])
                pl1 = switch(c1['away'], self.oR[c1['road_index']], self.oL[c1['road_index']])
                pt0 = pl0.eval(switch(c0['away'], 0, 1))[0] # end points nearest to turn
                pt1 = pl1.eval(switch(c1['away'], 0, 1))[0]
                
                # find intersection of polylines:
                intersection = curve_curve_intersection(pl0, pl1)
                
                if intersection is None: 
                    # No intersection was found, try again with extended curve ends. 
                    ext_dist = 10
                    pl0_ext = extend_curve(pl0, ext_dist, width1*extension) # note swapped widths and arbitrary factor
                    pl1_ext = extend_curve(pl1, ext_dist, width0*extension)

                    intersection = curve_curve_intersection(pl0_ext, pl1_ext)

                if intersection is not None: 
                    # check if too far away:
                    if math.dist(intersection, pt0) > width0*extension and math.dist(intersection, pt1) > width1*extension: 
                        intersection = None

                node.crv_intersections.append(intersection) # store the intersection point so the visible fillet generation can be generated quickly. None is fine too.
                
                if intersection is None:
                    # Trim both curves with separate circles
                    pl0_intersections = curve_curve_intersection(pl0, Circle(pt0, curb_radius))
                    pl1_intersections = curve_curve_intersection(pl1, Circle(pt1, curb_radius))
                else:
                    # trim normally
                    pl0_intersections = curve_curve_intersection(pl0, Circle(intersection, curb_radius))
                    pl1_intersections = curve_curve_intersection(pl1, Circle(intersection, curb_radius))


                if pl0_intersections:
                    if c0['away']: # get correct side
                        r0.trim_start[0] = pl0_intersections[-1] # note we want the furthest intersection from the circle
                    else:
                        r0.trim_end[1] = pl0_intersections[0]
                
                if pl1_intersections:
                    if c1['away']: # get correct side
                        r1.trim_start[1] = pl1_intersections[-1]
                    else:
                        r1.trim_end[0] = pl1_intersections[0]

        # Fix up the trim values sometimes being inverted:
        for road in self.roads:
            if road.trim_start[0] > road.trim_end[0]: # left side
                avg = (road.trim_start[0] + road.trim_end[0]) / 2
                road.trim_start[0] = avg
                road.trim_end[0] = avg
            
            if road.trim_start[1] > road.trim_end[1]: # right side
                avg = (road.trim_start[1] + road.trim_end[1]) / 2
                road.trim_start[1] = avg
                road.trim_end[1] = avg

            road.trim_dist_calculate()


    def generate_lane_mapping(self, angle_limit=120, mode_override=None):
        """All lanes exiting and approaching are intermediate and should not be used directly. Requires connections cache."""

        if 'lane_mapping' not in self.properties:
            self.properties.append('lane_mapping')
   
        for i, node in enumerate(self.nodes):
            
            # Generate cached lane lists per node:
            
            node.all_lanes_approaching = [] # list of lanes entering an intersection
            node.all_lanes_exiting = [] # list of lanes exiting an intersection
            for connection in node.connections:
                # Each connection branch gets its own set of branches
                road_lanes = self.roads[connection['road_index']].lanes
                #print(road_lanes)
                for lane_index, lane in enumerate(road_lanes):
                    if lane.type == 'carriageway':

                        lane_dict = {
                            'road_index': connection['road_index'],
                            'lane_index': lane_index,
                            'away': connection['away'],
                            'fwd': lane.fwd,
                            'bearing': connection['bearing'],
                            }

                        if lane.fwd == connection['away']:
                            node.all_lanes_exiting.append(lane_dict)
                        else:
                            node.all_lanes_approaching.append(lane_dict)
            
            # using above 2 cache lists, generate correct lane mappings (TODO consider custom mapping data)

            # Sort clockwise by bearing:
            node.all_lanes_approaching.sort(key=sort_by_bearing)
            node.all_lanes_exiting.sort(key=sort_by_bearing)

            # EVERYTHING ABOVE VERIFIED CORRECT

            if len(node.connections) == 1: # 1 connection, turn around / cul de sac
                connection = node.connections[0]
                
                distributed_lane_map = {approaching['lane_index']:[] for approaching in node.all_lanes_approaching} # create empty dict with keys and lists
                
                if node.all_lanes_exiting: # dead end

                    for approaching_index, approaching in enumerate(node.all_lanes_approaching):
                        exiting_index = int(round(remap(approaching_index, (0, len(node.all_lanes_approaching)-1), (len(node.all_lanes_exiting)-1, 0), True)))
                        distributed_lane_map[approaching['lane_index']].append(node.all_lanes_exiting[exiting_index])

                node.lane_mapping[connection['road_index']] = distributed_lane_map

            else:
                for connection in node.connections: # iterate over every node connection (needed because lane branches of a road are dependent on each other)
                    # Lanes consist of the following keys: road_index, lane_index, away, fwd, bearing. 
                    
                    # 1. Get relevant sorted lanes approaching and exiting, ensure those are in the angle limit:

                    lanes_approaching = [] # local list for all lanes part of the connection entering intersection
                    for lane in node.all_lanes_approaching:
                        if lane['road_index'] == connection['road_index']: # only those approaching from connection
                            lanes_approaching.append(dict(lane)) # copies of dict

                    lanes_exiting = [] # local list for all lanes reachable from the connection lane approached
                    for lane in node.all_lanes_exiting:
                        if lane['road_index'] != connection['road_index']: # only those not from the connection (this will exclude dead ends)
                            angle_diff = angle_difference(connection['bearing']-180, lane['bearing'])
                            if abs(angle_diff) <= angle_limit:
                                lane = dict(lane)
                                lane['bearing'] = angle_diff # local dict bearing now stores diff
                                lanes_exiting.append(lane) # copies of dict
                    
                    lanes_exiting.sort(key=sort_by_bearing) # sort by local bearing

                    if mode_override is None:
                        if len(lanes_approaching) == 1 and len(lanes_exiting) == 1:
                            map_mode = 'all' # lane changes allowed
                        else:
                            map_mode = 'weave' # intersection mapping
                    
                        if len(lanes_approaching) == 1:
                            map_mode = 'all' # a typical 1 lane each intersection
                    
                    else:
                        map_mode = mode_override
                    
                    # 2. distribute exiting amongst approaching, add to keys of lane index:
                    distributed_lane_map = distribution_map(
                        [appr['lane_index'] for appr in lanes_approaching], 
                        lanes_exiting, 
                        map_mode
                        )

                    # 3. add to lane mapping dict under correct road index:
                    node.lane_mapping[connection['road_index']] = distributed_lane_map # dict for lane index keys... road_index, lane_index, [exiting]

            #print('')
            #print(node.lane_mapping)

    def generate_mapping_curves(self):
        """Generate the curves connecting between lanes through a node. Store it in the same format as lane_mapping."""

        for node in self.nodes:
            for en_road_index, en_road in node.lane_mapping.items():
                for en_lane_index, en_lane in en_road.items():

                    n1 = self.roads[en_road_index].get_lane_end(en_lane_index, True)
                    
                    # now get the curves for each lane reachable:
                    for exiting in en_lane:
                        
                        n2 = self.roads[exiting['road_index']].get_lane_end(exiting['lane_index'], False)

                        cp = curve_curve_intersection(
                            Line(n1[0], move_pt(n1[0], n1[1], 20)), 
                            Line(n2[0], move_pt(n2[0], n2[1], -20))
                            )
                        
                        if cp is None:
                            crv = Line(n1[0], n2[0])

                        # Check to ensure the quadratic bezier doesn't have overlap:
                        elif math.dist(n1[0], cp) > TOL and math.dist(cp, n2[0]) > TOL and math.dist(n1[0], n2[0]) > TOL:
                            crv = QuadraticBezier(n1[0], cp, n2[0])
                        else:
                            crv = Line(n1[0], n2[0])
                        
                        key = (en_road_index, en_lane_index, exiting['road_index'], exiting['lane_index']) # 4-tuple, entering -> exiting
                        node.mapping_curves[key] = crv


    def generate_signalling(self):
        """Add default intersection details like stop signs and traffic lights. Requies lane mapping."""

        if 'signalling' not in self.properties:
            self.properties.append('signalling')

        for node in self.nodes:
            if node.connections == 2:
                continue # no intersection at all
            
            # rank each road by significance, take the best two

            road_significance = []
            #for connection in node.connections:

            

            #if node.connections == 3:
                # find best two road
            # Rules:
            # small roads get stop signs, two of them will get priority (longest curve)
            # if a
            #if random.random() < 0.3:
            #    node.ins_type = 'stop'
            
        pass


    def update_markers(self, dt):
        """Update all markers."""
        #print(len(self.markers))
        for m in self.markers:
            # only spawners have functionality to update
            if m.type == 'spawner_random' or m.type == 'spawner_pathfind':
                #print(m.timer)
                m.timer += dt # increase time
                
                if m.timer > m.spawn_spacing:
                    m.timer = 0 # reset timer to 0 again

                    
                    if m.spawns_remaining is not None:
                        m.spawns_remaining -= 1
                        if m.spawns_remaining <= 0:
                            return None # no spawns left

                    # spawn a vehicle with correct params:
                    # TODO random
                    #print(m.route[0])
                    chosen_route = random.randint(0, len(m.route)-1)
                    
                    self.add_vehicle(m.road_index, m.lane_indices[0], m.t, route=list(m.route[chosen_route])) # there can be multiple routes to choose from


    def create_vehicle_route(self, location, target=None, depth_limit=None):
        """Return a list of road and lane indices as a route starting from given location. If no target is provided, choose randomly."""
        #print(location, target, depth_limit)
        if target is None or target == (None, None): # no target, go somewhere random
            
            route = [] 
            road_index, lane_index = location # tuple to vars, these two vars can be changed as the search occurs
            
            if depth_limit == None:
                depth_limit = 1
            
            for _ in range(max(0, depth_limit)):
                road = self.roads[road_index]
                node_index = switch(road.lanes[lane_index].fwd, road.n2, road.n1)
                node = self.nodes[node_index] # get node at end of road
                accessible_lanes = node.get_lane_mapping(road_index, lane_index)

                if accessible_lanes: 
                    targeted_lane = random.choice(accessible_lanes) # choose random way to go

                    road_index, lane_index = targeted_lane['road_index'], targeted_lane['lane_index'] # update location
                    
                    route.append((road_index, lane_index))
                else:
                    break # dead end or failure of lane finder
            
            return route
        
        else: # search for target road and lane
            assert isinstance(target, tuple) or isinstance(target, list) # target should be road, lane.
            assert location != target # no where to go
            
            # optimisations: weighting by direction
            # delete roads that go into dead ends (unless start or end are in one)
            
            # this needs to consider each lane because the lanes have their own mappings, etc.
            
            # Create a pool of roads and lanes that can be modified:
            lane_pool = {}
            for r_i, road in enumerate(self.roads):
                lane_pool[r_i] = {} # needs to be a dict to remove it from the pool once visited? still needs to be known by visitable lanes
                for l_i, lane in enumerate(road.lanes):
                
                    node_start = switch(lane.fwd, road.n1, road.n2)
                    node_end = switch(lane.fwd, road.n2, road.n1)

                    lane_pool[r_i][l_i] = {
                        'road_index': r_i,
                        'lane_index': l_i,
                        'node_start': node_start, # might not be used?
                        'node_end': node_end,
                        'length': road.curve.length() / lane.speed, # effective length considering speed and possibly intersection
                        'dist_to_start': float('inf'), # editable: initial distance is infinity, measures to n2
                        'visited': False, # editable: whether this lane has ever been visited before (any prev road or lane)
                        'route': [], # a list of road,lane for how to navigate to here
                    }

            # start at first road, continually search around it, then go to next layer, and so on
            # now search from n2 of lane, ending at either n1 or n2 of destination:
            road_index, lane_index = location # starting road

            lane_pool[road_index][lane_index]['dist_to_start'] = 0 # first road has 0 distance
            #lane_pool[road_index][lane_index]['route'] = [(road_index, lane_index)] # start of route?

            if depth_limit == None:
                depth_limit = PATHFIND_DEPTH_LIMIT

            for depth_count in range(depth_limit):
                #print('pos:', road_index, lane_index)
                #print('route:', lane_pool[road_index][lane_index]['route'])
                # find all the roads it can lead to, calculate their distances:
                for accessible_lanes in self.nodes[lane_pool[road_index][lane_index]['node_end']].get_lane_mapping(road_index, lane_index):
                    r_i, l_i = accessible_lanes['road_index'], accessible_lanes['lane_index'] # temp vars
                    #print('check branch:', (r_i, l_i))
                    if r_i == target[0]: # target road found
                        if target[1] is None or l_i == target[1]:
                            print('found destination...')
                            
                            route = lane_pool[road_index][lane_index]['route']
                            route.append((r_i, l_i))
                            return route
                    
                    if not lane_pool[r_i][l_i]['visited']: # not visited, now update distance
                        #print('not visited')
                        new_dist = lane_pool[road_index][lane_index]['dist_to_start'] + lane_pool[r_i][l_i]['length']
                        
                        if new_dist < lane_pool[r_i][l_i]['dist_to_start']:
                            lane_pool[r_i][l_i]['dist_to_start'] = new_dist # shortest dist
                            
                            lane_pool[r_i][l_i]['route'] = list(lane_pool[road_index][lane_index]['route']) # must be a new object
                            lane_pool[r_i][l_i]['route'].append((r_i, l_i))


                # mark itself as visited:
                lane_pool[road_index][lane_index]['visited'] = True

                # get smallest dist to start that hasn't been visited:
                smallest_dist = float('inf')
                smallest_loc = None
                for road in lane_pool:
                    for lane in lane_pool[road]:
                        lane = lane_pool[road][lane]
                        if lane['dist_to_start'] < smallest_dist and not lane['visited']:
                            smallest_dist = lane['dist_to_start']
                            smallest_loc = (lane['road_index'], lane['lane_index'])

                if smallest_loc is None:
                    print(f'No smallest distance found. depth: {depth_count}, R{road_index}, L{lane_index}')
                    #print(lane_pool)
                    raise Exception('No smallest distance found.')

                road_index, lane_index = smallest_loc

            print('depth limit reached')
            return [] 


    def _process_vehicle_navigation(self):

        for v in self.vehicles:
            # check if position should make it change path
            if v.path_following == 'road':
                if (v.path_dir and v.t > v.path_t_end) or (not v.path_dir and v.t < v.path_t_end):
                    if v.behaviour == 'random': 
                        if v.route == []:
                            # generate basic, it was just spawned as random probably
                                v.route = self.create_vehicle_route((v.road_index, v.lane_index), depth_limit=5-len(v.route))
                        else:
                                for segment in self.create_vehicle_route(v.route[-1], depth_limit=5-len(v.route)):
                                    v.route.append(segment)
                    
                    if v.route == []: # nowhere to go
                        v.delete_flag = True
                        print(f'no lanes for R{v.road_index} L{v.lane_index}')
                        continue
                    
                    
                    previous_road = self.roads[v.road_index]
                    
                    node_index = switch(previous_road.lanes[v.lane_index].fwd, previous_road.n2, previous_road.n1)
                    v.path = self.nodes[node_index].mapping_curves[(v.road_index, v.lane_index, v.route[0][0], v.route[0][1])]
                    
                    # Update location using next item and delete it from route:
                    v.prev_road_index = v.road_index
                    v.prev_lane_index = v.lane_index
                    v.road_index = v.route[0][0]
                    v.lane_index = v.route[0][1]
                    v.route.pop(0) # remove the now entered road
                    
                    # Update the rest of the data:
                    targeted_road = self.roads[v.road_index]
                    
                    v.path_offset = 0
                    v.path_dir = True
                    v.path_length = v.path.length()

                    v.path_t_start = 0
                    v.path_t_end = 1
                    v.t = 0
                    v.path_following = 'node'
                    v.road_speed_limit = targeted_road.lanes[v.lane_index].speed
                    

            elif v.path_following == 'node':
                if v.t > 1: # node paths are always t 0 -> 1
                    targeted_road = self.roads[v.road_index]

                    v.path = targeted_road.curve
                    v.path_offset = targeted_road.get_x_offset(v.lane_index)
                    v.path_dir = targeted_road.lanes[v.lane_index].fwd
                    v.path_length = v.path.length()
                    lane_trim = targeted_road.get_trim(v.path_offset)

                    v.path_t_start = switch(v.path_dir, lane_trim[0], lane_trim[1])
                    v.path_t_end = switch(v.path_dir, lane_trim[1], lane_trim[0])
                    v.t = switch(v.path_dir, lane_trim[0], lane_trim[1])
                    v.path_following = 'road'
                    
            else:
                #print('unknown path type')
                raise Exception('unknown path type')

            ################


    def _get_spd_tgts_road(self, road_index, lane_index, start_dist):
        """Get all speed targets. Includes vehicles and end signalling. 
        Return a list of targets. start_dist is used to offset the distances."""

        road = self.roads[road_index]
        road: RoadSegment
        trim = road.get_trim(road.get_x_offset(lane_index))
        length = road.curve.length() * (trim[1]-trim[0])
        
        speed_limit = self.roads[road_index].lanes[lane_index].speed

        tgts = []
        
        # get mid speeds, should scale depending on length
        tgts.append(('limit', start_dist+0.1*length, speed_limit))
        if length > 40: # extra spds for long roads
            tgts.append(('limit', start_dist+0.3*length, speed_limit))
            tgts.append(('limit', start_dist+0.7*length, speed_limit))
        tgts.append(('limit', start_dist+0.9*length, speed_limit))

        for other_vehicle in self.vehicle_map.get(road_index, {}):
            other_vehicle: Vehicle
            if other_vehicle.lane_index == lane_index and other_vehicle.path_following == 'road':
                if other_vehicle.speed is not None:
                    tgts.append(('vehicle', start_dist+other_vehicle.distance_along_path, other_vehicle.speed*0.8))

        node: Node = self.nodes[switch(road.lanes[lane_index].fwd, road.n2, road.n1)]
        if node.ins_type == 'stop':
            # TODO check if any vehicle is in the way, otherwise continue through the intersection
            tgts.append(('stop', start_dist+length, 0))


        return tgts


    def _get_spd_tgts_node(self, road_index, lane_index, start_dist, prev_road_index=None, prev_lane_index=None):
        """Get all speed targets. Returns curve as well."""
        # first get node
        r1: RoadSegment = self.roads[road_index]
        node: Node = self.nodes[switch(r1.lanes[lane_index].fwd, r1.n1, r1.n2)]
        
        tgts = []

        for other_vehicle in self.vehicle_map.get(road_index, {}):
            other_vehicle: Vehicle
            if other_vehicle.lane_index == lane_index and other_vehicle.path_following == 'node':
                if other_vehicle.speed is not None:
                    tgts.append(('vehicle', start_dist+other_vehicle.distance_along_path, other_vehicle.speed*0.8))

        if prev_road_index is None:
            return tgts, None # TODO ####

        crv = node.mapping_curves[(prev_road_index, prev_lane_index, road_index, lane_index)]
        #crv: QuadraticBezier
        #crv.curvature(0.5)
        tgts.append(('limit', start_dist+crv.length()/2, kph(10))) # turn max curvature
        
        return tgts, crv

    

    def update_vehicles(self, dt):
        """Update all vehicles."""
        print('tick', len(self.vehicles))
        # Generate a cache to more efficiently search the road network vehicles. 
        # Keys indicate road index, values are vehicle objects on those roads.
        self.vehicle_map = {}
        for v in self.vehicles:
            if v.road_index in self.vehicle_map:
                self.vehicle_map[v.road_index].append(v)
            else:
                self.vehicle_map[v.road_index] = [v]


        # Cache individual vehicle data first (ordering because of spawns):
        for v in self.vehicles:
            #print(v.path_t_start, v.path_t_end)
            v.distance_along_path = switch(v.path_dir, v.t-v.path_t_start, v.path_t_start-v.t) * v.path_length # measuring from t start
        

        # MAIN MOVEMENT
        for v in self.vehicles:
            #print(v.t, v.prev_road_index, v.prev_lane_index, v.road_index, v.lane_index)
            dist_to_start = 0-v.distance_along_path#-FWD_STOP_BUFFER
            #print(dist_to_start)
            v.tgt_speed_graph = []

            # DEBUG
            #v.tgt_speed_graph.append(('stop', v.path_length-v.distance_along_path, 0))
            
            # node entering road
            if v.path_following == 'node':
                for tgt in self._get_spd_tgts_node(v.road_index, v.lane_index, dist_to_start, v.prev_road_index, v.prev_lane_index)[0]:
                    v.tgt_speed_graph.append(tgt)
                dist_to_start += v.path_length
            
            # road segment
            for tgt in self._get_spd_tgts_road(v.road_index, v.lane_index, dist_to_start):
                v.tgt_speed_graph.append(tgt)
            dist_to_start += self.roads[v.road_index].curve.length() * abs(v.path_t_end - v.path_t_start)

            # next node
            prev_road_index = v.road_index
            prev_lane_index = v.lane_index
            #print(prev_road_index, prev_lane_index)
            #print(v.route)
            for road_index, lane_index in v.route:
                node_spd_tgts = self._get_spd_tgts_node(road_index, lane_index, dist_to_start, prev_road_index, prev_lane_index)
                for tgt in node_spd_tgts[0]:
                    v.tgt_speed_graph.append(tgt)
                dist_to_start += node_spd_tgts[1].length()
                
                for tgt in self._get_spd_tgts_road(road_index, lane_index, dist_to_start):
                    v.tgt_speed_graph.append(tgt)
                dist_to_start += self.roads[road_index].curve.length() * abs(v.path_t_end - v.path_t_start)
                
                prev_road_index = road_index
                prev_lane_index = lane_index

                if dist_to_start > ROUTE_TGT_DIST_LIMIT: # length limit
                    break
            
            if not v.route: # end of road, just go to max speed
                v.tgt_speed_graph.append(('stop', dist_to_start+10, v.road_speed_limit))

            
            #print(v.tgt_speed_graph)
            #v.tgt_speed_graph.append((v.path_length-v.distance_along_path, 0))
            ################
            #print(v.tgt_speed_graph)
            v.tgt_speed_graph.sort(key=lambda x: x[1])
            #print(v.tgt_speed_graph)
            #print('__')
            decay_fac = 0.1 # 0.01 = 100m
            tgt_accel = 0
            
            avg_accel = 0
            interpolate = 1
            
            for tgt_type, dist, spd in v.tgt_speed_graph:
                if tgt_type == 'vehicle':
                    if dist-VEHICLE_MIN_SPACING > 0:
                        tgt_accel = const_accel(v.speed, spd, dist-VEHICLE_MIN_SPACING)
                        
                        if tgt_accel < -2: # max braking, highest priority rule
                            avg_accel = tgt_accel
                            #print('slow')
                            break
                        
                        avg_accel += interpolate*(tgt_accel-avg_accel)
                        interpolate = min(max(0, (1/(dist*decay_fac))), 1)

                    elif dist > 0:
                        #print('stop')
                        v.speed = 0 # went too close to the vehicle
                        avg_accel = 0
                        break
                
                elif tgt_type == 'stop': # must stop, must not go beyond this limit
                    
                    if dist-FWD_STOP_BUFFER > 0: 
                        tgt_accel = const_accel(v.speed, spd, dist-FWD_STOP_BUFFER)
                        
                        if tgt_accel < -2: # max braking, highest priority rule
                            avg_accel = tgt_accel
                            #print('slow', round(dist, 3))
                            break
                    
                    elif dist > 0:
                        #print('stop')
                        v.speed = 0 # went past the line
                        avg_accel = 0

                        break
                
                else:
                    if dist > 0: 
                        # need to calculate correct target accel based on current speed and dist to target
                        tgt_accel = const_accel(v.speed, spd, dist)
                        
                        if tgt_accel < -2: # max braking, highest priority rule
                            #avg_accel = -2
                            avg_accel = tgt_accel
                            break
                        
                        else:
                            avg_accel += interpolate*(tgt_accel-avg_accel)
                            interpolate = min(max(0, (1/(dist*decay_fac))), 1)
                    
            
            #print(f'spd:{round(v.speed, 2)} dist:{round(dist, 2)} ta:{round(tgt_accel, 2)}, td:{round(dist, 2)}', end='\r')
            
            # Acceleration and speed:
            v.acceleration = min(max(-3, avg_accel), 2) # max possible accel clamp
            v.speed = max(0, v.speed + dt*v.acceleration)

            if v.speed < 0.01: # snap to 0 to prevent approach to infinity
                v.speed = 0

            # Update speed and pos in the right direction:
            v.t_speed = v.speed / v.path_length
            v.t += dt*switch(v.path_dir, v.t_speed, -v.t_speed)

            # Visible location and dir (vec):
            path_eval = v.path.eval(v.t)
            if v.path_dir:
                v.pos = offset_pt(path_eval[0],path_eval[1],v.path_offset)
                v.dir = path_eval[1]
            else:
                v.pos = offset_pt(path_eval[0],path_eval[1],v.path_offset)
                v.dir = reverse_vec(path_eval[1])


        ##########
        
        self._process_vehicle_navigation()
        
        # DELETE VEHICLES
        i = 0
        for _ in range(len(self.vehicles)):
            if self.vehicles[i].delete_flag:
                self.vehicles.pop(i)
            else:
                i+=1


    def data(self):
        """Return packed network data based on inputted data.\n
        Dictionary/object for nodes and roads. No custom Python objects."""
        
        data = {
            'info':{
                'node_count':len(self.nodes),
                'road_count':len(self.roads),
                'properties':self.properties,
                }
            }
        data['nodes'] = [x.data() for x in self.nodes]
        data['roads'] = [x.data() for x in self.roads]
        data['markers'] = [x.data() for x in self.markers]
        data['vehicles'] = [x.data() for x in self.vehicles]
        
        return data
    
    def save_data_frame(self):
        """Trigger all vehicles to save their current state in their own animation data."""
        if self.time < self.record_duration and self.time >= 0:
            for v in self.vehicles:
                v.save_data_frame(self.time)
    
    def analysis_frame(self, dt):
        """Run any temporal analysis. Store them in the road segment."""

        for v in self.vehicles:
            if v.path_following == 'road':
                road = self.roads[v.road_index]
                
                road.analysis_occupancy_time_total += dt

                road.analysis_avg_speed += v.speed / road.analysis_avg_speed_elements
                road.analysis_avg_speed_elements += 1

    def get_animation(self):
        """Return a list of vehicle animation data."""
        generated_animation = []

        for v in self.vehicles:
            generated_animation.append(v.animation_data())
        
        return generated_animation

    def get_analysis(self):
        """Get analysis data."""
        analysis = {key:[] for key in [
            'occupancy_time_total', 'avg_speed'
            ]}

        for r in self.roads:
            analysis['occupancy_time_total'].append(r.analysis_occupancy_time_total)
            analysis['avg_speed'].append(r.analysis_avg_speed)

        return analysis
    

    def get_fillets(self, as_json=False):
        """Generate fillet data and return as a list of lists. Set `as_json` to True to get a JSON representation."""

        fillets = [] # store the curves here
        
        
        for node in self.nodes: 
            node_fillets = []
            for i0, i1 in CyclicPair(len(node.connections)):
                
                r0 = self.roads[node.connections[i0]['road_index']]
                r1 = self.roads[node.connections[i1]['road_index']]

                if node.connections[i0]['away']: # first road away, use start L
                    trim = r0.curve.eval(r0.trim_start[0])
                    r0_trim = (trim[0], reverse_vec(trim[1]))
                else: # first road approach, use end R
                    r0_trim = r0.curve.eval(r0.trim_end[1])

                if node.connections[i1]['away']: # second road away, use start R
                    trim = r1.curve.eval(r1.trim_start[1])
                    r1_trim = (trim[0], reverse_vec(trim[1]))
                else: # second road approach, use end L
                    r1_trim = r1.curve.eval(r1.trim_end[0])

                n_start = offset_pt(r0_trim[0], r0_trim[1], r0.get_width()*0.5)
                n_end = offset_pt(r1_trim[0], r1_trim[1], r1.get_width()*-0.5)
            
                if node.crv_intersections[i0] is None:
                    if n_start != n_end: # check if not degenerate
                        if as_json:
                            node_fillets.append(Line(n_start, n_end).data())
                        else:
                            node_fillets.append(Line(n_start, n_end))
                else:
                    if n_start != n_end: # check if not degenerate
                        if n_start != tuple(node.crv_intersections[i0]) and n_end != tuple(node.crv_intersections[i0]):
                            if as_json:
                                node_fillets.append(QuadraticBezier(n_start, node.crv_intersections[i0], n_end).data())
                            else:
                                node_fillets.append(QuadraticBezier(n_start, node.crv_intersections[i0], n_end))
            
            fillets.append(node_fillets)
        
        return fillets

##############################

def set_seed(seed=None):
    """Set seed."""
    random.seed(seed) # note that None will use system seed as usual


def load_packed_network(data):
    """Create and return a Network object based on inputted data.\n
    Dictionary/object for nodes and roads. No custom Python objects."""
    
    network = RoadNetwork()

    # Properties: list of properties of the network
    if 'properties' in data['info']:
        network.properties = data['info']['properties']
    else:
        print('no properties found')
    

    # Nodes: list of dicts for coordinates
    for node in data['nodes']:
        new_node = Node(node['x'], node['y'])
        if 'z' in node:
            new_node.z = node['z']
        if 'connections' in node:
            new_node.connections = node['connections']
        if 'crv_intersections' in node:
            new_node.crv_intersections = node['crv_intersections']
        if 'lane_mapping' in node:
            #new_node.lane_mapping = node['lane_mapping']
            
            # clean up the mapping so it uses ints as keys
            new_dict = {}

            for key_road in node['lane_mapping']:
                for key_lane in node['lane_mapping'][key_road]:
                    if int(key_road) in new_dict:
                        new_dict[int(key_road)][int(key_lane)] = node['lane_mapping'][key_road][key_lane]
                    else:
                        new_dict[int(key_road)] = {int(key_lane): node['lane_mapping'][key_road][key_lane]}

            new_node.lane_mapping = new_dict

        network.nodes.append(new_node)

    # Roads: list of roads containing dicts for params
    for road in data['roads']:
        if 'lanes' in road:
            if road['lanes']: # empty list check
                lanes = [Lane(l.get('type','carriageway'), l.get('width',3), l.get('fwd',True), l.get('speed',kph(50))) for l in road['lanes']]
            else:
                lanes = None
        else:
            lanes = None
        
        n1 = road['n1']
        n2 = road['n2']
        trim_start = road.get('trim_start', (0,0))
        trim_start = (trim_start[0], trim_start[1])
        trim_end = road.get('trim_end', (1,1))
        trim_end = (trim_end[0], trim_end[1])

        road_name = road.get('road_name', None)

        if 'curve' in road:
            curve = road['curve'] # set curve to dict of curve data temporarily
        else:
            continue # required

        if curve['crv_type'] == 'Line':
            if curve['n1'] == curve['n2']:
                print('skip: degenerate line curve') # can't share same node
                continue 
            curve = Line(curve['n1'], curve['n2'])
        
        elif curve['crv_type'] == 'PolyLine':
            curve = PolyLine(curve['pts'])
        
        elif curve['crv_type'] == 'QBezier':
            if curve['n1'] == curve['n2']:
                print('skip: degenerate line curve') # can't share same node
                continue 
            curve = QuadraticBezier(curve['n1'], curve['cp'], curve['n2'])
        
        else:
            print(curve['crv_type'])
            continue # unrecognised curve type, don't create a road segment!

        # create a road segment:

        new_road_segment = RoadSegment(n1, n2, curve, lanes, trim_start, trim_end, road_name)
        network.roads.append(new_road_segment)


    # Markers: 
    for marker in data['markers']:
        if marker['road_index'] >= len(network.roads):
            raise Exception('Marker index out of range')
        
        new_marker = Marker(marker['road_index'], marker['lane_indices'], marker['t'], marker['type'])

        new_marker.name = marker['name']
        new_marker.spawn_spacing = marker['spawn_spacing']
        new_marker.timer = new_marker.spawn_spacing
        new_marker.spawns_remaining = marker['spawns_remaining']
        new_marker.route = marker['route']

        network.markers.append(new_marker)


    # Vehicles: 
    for vehicle in data['vehicles']:
        """'path_following':self.path_following,
            """ # TODO

        new_vehicle = Vehicle(vehicle['road_index'], vehicle['lane_index'], vehicle['t'])
        
        new_vehicle.route = vehicle['route']
        new_vehicle.behaviour = vehicle['behaviour']
        new_vehicle.speed = vehicle['speed']
        
        new_vehicle.initialise_data(network.roads[new_vehicle.road_index])
        
        network.vehicles.append(new_vehicle)


    return network

#############################

if __name__ == '__main__':
    print('TESTS:\n')
    
    network = RoadNetwork()
    
    network.add_node(0, 0)
    network.add_node(40, 0)
    network.add_node(40, 40)
    network.add_node(0, 30)

    network.add_road(0, 1)
    network.add_road(1, 2)
    network.add_road(2, 3)
    network.add_road(3, 0)
    #network.add_road(0, 2)
    
    print('>>')
    network.generate_connection_cache()
    network.trim_roads()
    network.generate_lane_mapping()

    
    #print(network.create_vehicle_route((0,1), depth_limit=4))
    #print(network.create_vehicle_route((0,1), (4,None)))

    #print(kph(110))
    """test = RoadSegment(0, 1, None, ROAD_GENERIC)
    print(test.get_width())
    print(test.get_x_offset(0))
    print(test.get_x_offset(1))
    print(test.get_x_offset(2))
    print(test.get_x_offset(3))"""

    #import json
    #print(network.data())
    #with open('test.json', 'w') as f:
    #    f.write(json.dumps(network.data(), indent=2))
    print('>>done')