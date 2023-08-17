import pygame
from curve import Line, QuadraticBezier, PolyLine, offset_curve
import network as N
import math
from utils import *
import json
import random
from pygame_utils import *


def ct(x, y=None):
    """Camera Transform. Get screen space coordinates."""
    global cam_scale, cam_x, cam_y, screen_offset_x, screen_offset_y
    if y is None:
        y = x[1]
        x = x[0]
    return ((x-cam_x)*cam_scale + screen_offset_x, (0-y-cam_y)*cam_scale + screen_offset_y)


def on_screen(point, margin=0):
    """Check if a point is on the screen."""
    if -margin < ct(point)[1] < screen.get_height()+margin:
        if -margin < ct(point)[0] < screen.get_width()+margin:
            return True
    return False


def draw_vec(root, vec, scale=1, colour='#ff00ff', style='simple'):
    """Draw a vector given a root point and a vector."""

    if style == 'simple':
        pygame.draw.aaline(screen, colour, ct(root), ct(root[0]+vec[0]*scale, root[1]+vec[1]*scale))
        pygame.draw.circle(screen, colour, ct(root), 3)
    elif style == 'arrow':
        tip = root[0]+vec[0]*scale, root[1]+vec[1]*scale
        vec = vec_2pts(root, tip)
        
        pygame.draw.circle(screen, colour, ct(root), 3)
        pygame.draw.aaline(screen, colour, ct(root), ct(tip))
        pygame.draw.aaline(screen, colour, ct(tip), ct(offset_pt_2D(tip, vec, -0.2, -0.25)))
        pygame.draw.aaline(screen, colour, ct(tip), ct(offset_pt_2D(tip, vec, 0.2, -0.25)))
        

def draw_dot(pt, radius=2, colour='#ff00ff'):
    pygame.draw.circle(screen, colour, ct(pt), radius)

def draw_cross(pt, radius=1, colour='#ff00ff'):
    pygame.draw.line(screen, colour, ct(move_pt(pt,(-radius,-radius))), ct(move_pt(pt,(radius,radius))))
    pygame.draw.line(screen, colour, ct(move_pt(pt,(-radius,radius))), ct(move_pt(pt,(radius,-radius))))

def draw_curve(curve, colour='#ff00ff', offset=0, width=1, t_start=0, t_end=1):
    """Draw Curve."""

    # t limits
    if t_start >= t_end:
        if t_start == t_end:
            return None
        else:
            raise Exception('t_start larger than t_end')
    
    t_start = max(0,t_start)
    t_end = min(t_end,1)

    if isinstance(curve, Line):
        # line, just use endpoints
        n1, tangent = curve.eval(t_start)
        n2 = curve.eval(t_end)[0]

        pygame.draw.line(screen, colour, ct(offset_pt(n1, tangent, offset)), ct(offset_pt(n2, tangent, offset)), width=width)

    elif isinstance(curve, QuadraticBezier):
        STEPS = 10
        for t in range(STEPS):
            t0 = t/STEPS # root t
            t1 = (t+1)/STEPS # next t
            d0 = curve.eval(t0)
            d1 = curve.eval(t1)

            if t0 >= t_start and t1 <= t_end: # mid (normal)
                pygame.draw.line(screen, colour, ct(offset_pt(d0[0], d0[1], offset)), ct(offset_pt(d1[0], d1[1], offset)), width=width)
            
            elif t0 < t_start and t1 > t_start: # start
                t_new = curve.eval(t_start)
                pygame.draw.line(screen, colour, ct(offset_pt(t_new[0], t_new[1], offset)), ct(offset_pt(d1[0], d1[1], offset)), width=width)
            
            elif t0 < t_end and t1 > t_end: # end
                t_new = curve.eval(t_end)
                pygame.draw.line(screen, colour, ct(offset_pt(d0[0], d0[1], offset)), ct(offset_pt(t_new[0], t_new[1], offset)), width=width)

            elif t0 < t_start and t1 > t_end: # both ends are close together
                t_new1 = curve.eval(t_start)
                t_new2 = curve.eval(t_end)
                pygame.draw.line(screen, colour, ct(offset_pt(t_new1[0], t_new1[1], offset)), ct(offset_pt(t_new2[0], t_new2[1], offset)), width=width)
            
        #pygame.draw.line(screen, colour, ct(curve.n1), ct(curve.cp))
        #pygame.draw.line(screen, colour, ct(curve.cp), ct(curve.n2))
        
    
    elif isinstance(curve, PolyLine):
        pts = []
        d = curve.eval(t_start)
        pts.append(offset_pt(d[0], d[1], offset))

        for i, t in enumerate(curve.t_values):
            if t_end > t > t_start:
                d = curve.eval(t)
                pts.append(offset_pt(d[0], d[1], offset))

        d = curve.eval(t_end)
        pts.append(offset_pt(d[0], d[1], offset))
        
        for i in range(len(pts)-1):
            pygame.draw.line(screen, colour, ct(pts[i]), ct(pts[i+1]), width=width)
            
    else:
        raise Exception('Unknown curve type')


def camera(movement_speed=1000):
    keys = pygame.key.get_pressed()
    global cam_x, cam_y, cam_scale
    if keys[pygame.K_w]:
        cam_y -= (movement_speed * dt) / cam_scale
    if keys[pygame.K_s]:
        cam_y += (movement_speed * dt) / cam_scale
    if keys[pygame.K_a]:
        cam_x -= (movement_speed * dt)  / cam_scale
    if keys[pygame.K_d]:
        cam_x += (movement_speed * dt)  / cam_scale

    if keys[pygame.K_e]:
        cam_scale *= 1+1*dt
        if cam_scale > 16:
            cam_scale = 16

    if keys[pygame.K_q]:
        cam_scale /= 1+1*dt
        if cam_scale < 1/16:
            cam_scale = 1/16

def draw_refs():
    """Reference markers"""
    if DARK_MODE:
        colour = '#404040'
    else:
        colour = '#e0e0e0'

    pygame.draw.aaline(screen, colour, ct(0,-100), ct(0,100), 2)
    pygame.draw.aaline(screen, colour, ct(-100,0), ct(100,0), 2)
    draw_dot((0,0), 4, colour)


# SIMULATION SETUP 

SIMULATION_TIMESTEP = 1/30
CORNER_RADIUS = 3
LOAD = 4
DARK_MODE = True
VEHICLES = 0
START_GAME = True

RENDER_LAYERS = {
    'screen_size_large':False,
    'debug':True,
    'refs':True,
    'road_edges':True,
    'road_edges_polylines':False,
    'road_curves':False,
    'lane_vectors':True,
    'lane_ends':False,
    'trim_circles':False,
    'trim_dots_small':False,
    'filled_surfaces':False,
    'fillets':True,
    'vehicles':True,
    'speed_tgts':True,
    'hovered_content':True,
}

TGT_COL = {
    'stop':'#ff0000',
    'limit':'#80ff80',
    'vehicle':'#00ffff',
}

if LOAD == 0:
    network = N.RoadNetwork()
    network.add_node(-50,-20)
    network.add_node(40,10)
    network.add_road(0,1,QuadraticBezier((-50,-20),(-5,60),(40,10)),None)

if LOAD == 1:
    network = N.RoadNetwork()
    network.add_node(-3, 3)
    network.add_node(100, -35)
    network.add_node(130, 100)
    network.add_node(0, 100)
    network.add_node(-90, 20)
    network.add_node(-75, -76)

    network.add_road(0, 1, lanes=N.ROAD_ALLEY)
    network.add_road(3, 0)
    network.add_road(2, 0,  lanes=N.ROAD_STREET)
    network.add_road(0, 4)
    network.add_road(0, 5)

if LOAD == 2:
    with open('data/synthesised1.json') as f:
        network = N.load_packed_network(json.loads(f.read()))

if LOAD == 3:
    with open('data/testing.json') as f:
        network = N.load_packed_network(json.loads(f.read()))

if LOAD == 4:
    with open('data/five_dock.json') as f:
        network = N.load_packed_network(json.loads(f.read()))

if LOAD == 5:
    network = N.RoadNetwork()
    network.add_node(0, 0)
    network.add_node(80, 0)
    network.add_node(80, 80)
    network.add_node(20, 80)

    network.add_road(0,1, lanes=N.ROAD_GENERIC)
    network.add_road(1,2, lanes=N.ROAD_HIGHWAY)
    network.add_road(2,3, lanes=N.ROAD_STREET)
    network.add_road(3,0, lanes=N.ROAD_ALLEY)

if LOAD == 6:
    network = N.RoadNetwork()
    network.add_node(0,0)
    network.add_node(80,0)
    network.add_road(0,1)
    network.add_vehicle(0,1,0.5, speed=12)

N.set_seed(0) # seed for PRNG

network.generate_connection_cache() # which roads connect where?
network.trim_roads(15, CORNER_RADIUS)
network.generate_lane_mapping() # automatic lanes
network.generate_mapping_curves() # curves for vehicles to follow
network.add_random_traffic(VEHICLES) # add random vehicles
network.generate_signalling()

print(len(network.vehicles), 'spawned')

#######################################################################
# PYGAME

if START_GAME:
    pygame.init()
    pygame.display.set_caption("Traffic Simulator")
    if RENDER_LAYERS['screen_size_large']:
        screen = pygame.display.set_mode((1920, 1080))
    else:
        screen = pygame.display.set_mode((1280, 720))

    clock = pygame.time.Clock()
    running = True
    dt = 0
    simulation_deviation = 0

    screen_offset_x, screen_offset_y = screen.get_width() / 2, screen.get_height() / 2
    cam_x, cam_y = 0, 0
    cam_scale = 5

    last_log = None
    route = []
    route_geo = []

    cached_fillets = network.get_fillets()

else:
    running = False


while running:
    # PYGAME EVENTS
    for event in pygame.event.get():
        if event.type == pygame.QUIT: # window closed (x clicked)
            running = False

        if event.type == pygame.KEYUP and event.key == pygame.K_SPACE:
            if hovered_content[0] == 'node':
                
                print(hovered_content[1])
                #print(network.nodes[hovered_content[1]].all_lanes_approaching) 
                #print('')
                #print(network.nodes[hovered_content[1]].all_lanes_exiting) 

            elif hovered_content[0] == 'connection':
                print(hovered_content[3])
                #print(node.get_lane_mapping(, 0))
            
            elif hovered_content[0] == 'road':

                lanes = network.roads[hovered_content[1]].get_navigable_lanes()
                if lanes:
                    chosen_lane = random.choice(lanes)
                    
                    network.add_vehicle(hovered_content[1],chosen_lane,0.5)#, speed=remap(random.random(), output_domain=(0,14)))
                    print('spawn vehicle')
                else:
                    print('no space')
                
        if event.type == pygame.KEYUP and event.key == pygame.K_1:
            if hovered_content[0] == 'road':
                
                lanes = network.roads[hovered_content[1]].get_navigable_lanes()
                if lanes:
                    chosen_lane = random.choice(lanes)
                    route_start = (hovered_content[1], 0) # no random
                    print('set route start', route_start)
                else:
                    print('no space')

        if event.type == pygame.KEYUP and event.key == pygame.K_0:
            network.vehicles = []

        if event.type == pygame.KEYUP and event.key == pygame.K_2:
            """print(route_start)
            route = network.create_vehicle_route(route_start, depth_limit=5)
            route.insert(0, route_start)
            print(route)"""
            
            try:
                
                if hovered_content[0] == 'road':
                    route_end = (hovered_content[1], None)
                    print(route_start, route_end)
                    route = network.create_vehicle_route(route_start, route_end)
                
                    route.insert(0, route_start)
                    print(route)

                    route_geo = []
                    for r, l in route:
                        
                        route_geo.append(offset_curve(network.roads[r].curve, network.roads[r].get_x_offset(l), steps=7))
                        
            except:
                print('failure')
                
        if event.type == pygame.KEYUP and event.key == pygame.K_z:
            print(hovered_content)


    # SIMULATION

    repeat = int(simulation_deviation // SIMULATION_TIMESTEP)
    if repeat > 10:
        print('significant time deviation', simulation_deviation)
        repeat = 10
        simulation_deviation = SIMULATION_TIMESTEP * repeat # reset deviation

    for _ in range(repeat):
        network.update_markers(SIMULATION_TIMESTEP)
        if RENDER_LAYERS['vehicles']:
            network.update_vehicles(SIMULATION_TIMESTEP)
            #network.save_data_frame()
            network.time += SIMULATION_TIMESTEP
        simulation_deviation -= SIMULATION_TIMESTEP

    # MISC
    hovered_content = (None, None)
    camera()

    # CLEAR SCREEN
    if DARK_MODE:
        screen.fill("#121212")
    else:
        screen.fill("#ffffff")
    
    if RENDER_LAYERS['refs']:
        draw_refs()
    
    # DRAW DEBUG TRIM CIRCLES
    if RENDER_LAYERS['trim_circles']:
        for node in network.nodes: 
            for pt in node.crv_intersections:
                if pt is not None:
                    draw_dot(pt, cam_scale*CORNER_RADIUS, '#905050')


    # DRAW NODES
    for node_index, node in enumerate(network.nodes):
        if on_screen(node.pos(), 10):
            if hovered_content[0] is None:
                if 10 > math.dist(pygame.mouse.get_pos(), ct(node.pos())):
                    hovered_content = ('node', node_index)
                    
        
            for conn_index, connection in enumerate(node.connections):
                dot_pos = move_pt(node.pos(), connection['vec'], 10)

                if node.ins_type == 'stop':
                    draw_dot(dot_pos, 3, '#ff0000')
                else:
                    draw_dot(dot_pos, 3, '#603000')
                
                if hovered_content[0] is None:
                    if 10 > math.dist(pygame.mouse.get_pos(), ct(dot_pos)):
                        hovered_content = ('connection', node_index, conn_index, connection)
                        draw_dot(dot_pos, 5, '#ffffff')

                        # draw lane mapping
            
            for key, crv in node.mapping_curves.items():
                draw_curve(crv, '#520000')

    # DRAW ROADS & OFFSET CURVES
    for road_index, road in enumerate(network.roads):

        #print(road.curve)
        width = road.get_width()

        # get the lanes and draw lines
        if RENDER_LAYERS['filled_surfaces']:
            for l, lane in enumerate(road.lanes):
                if lane.type == 'carriageway':
                    colour = '#353535'
                else:
                    colour = '#808080'
                offset = road.get_x_offset(l)
                draw_curve(road.curve, colour, offset, round(cam_scale*lane.width*0.8), t_start=road.get_trim(offset)[0], t_end=road.get_trim(offset)[1])
        
        # lane end points
        if RENDER_LAYERS['lane_ends']:
            for l, lane in enumerate(road.lanes):
                end = road.get_lane_end(l, False)
                draw_vec(end[0], end[1], 5, '#00f0ff')
                
                end = road.get_lane_end(l, True)
                draw_vec(end[0], end[1], 5, '#00f040')

        # vectors
        if RENDER_LAYERS['lane_vectors']:
            for l, lane in enumerate(road.lanes):
                if lane.type == 'carriageway':
                    middle = road.curve.eval(0.5)
                    draw_vec(offset_pt(middle[0], middle[1], road.get_x_offset(l)), middle[1], scale=switch(lane.fwd, 10, -10), colour='#00ffff', style='arrow')


        # outlines
        if RENDER_LAYERS['road_curves']:
            draw_curve(road.curve, '#ffffff', t_start=0, t_end=1)
        
        if RENDER_LAYERS['road_edges']:
            draw_curve(road.curve, '#ff4040', width*-0.5, 1, road.trim_start[0], road.trim_end[0]) # left side
            draw_curve(road.curve, '#40ff40', width*0.5, 1, road.trim_start[1], road.trim_end[1]) # right side
        
        if RENDER_LAYERS['road_edges_polylines']:
            draw_curve(network.oL[road_index], '#ff4040') # offset polyline version
            draw_curve(network.oR[road_index], '#40ff40')

        if hovered_content[0] is None:
            if 30 > dist_pt_to_line_segment(pygame.mouse.get_pos(), ct(network.nodes[road.n1].pos()), ct(network.nodes[road.n2].pos())):
                hovered_content = ('road', road_index)
        
        if RENDER_LAYERS['trim_dots_small']:
            # dots at trim
            trim = road.curve.eval(road.trim_start[0])
            draw_dot(offset_pt(trim[0], trim[1], width*-0.5), 2, '#ff0000')
            trim = road.curve.eval(road.trim_end[0])
            draw_dot(offset_pt(trim[0], trim[1], width*-0.5), 2, '#ff0000')
            trim = road.curve.eval(road.trim_start[1])
            draw_dot(offset_pt(trim[0], trim[1], width*0.5), 2, '#00ff00')
            trim = road.curve.eval(road.trim_end[1])
            draw_dot(offset_pt(trim[0], trim[1], width*0.5), 2, '#00ff00')
    
    
    # DRAW FILLETS
    if RENDER_LAYERS['fillets']:
        for node_fillets in cached_fillets:
            for fillet in node_fillets:
                if on_screen(fillet.n1):
                    if fillet.n1 != fillet.n2:
                        draw_curve(fillet, '#ffff00')


    # DEBUG GEOMETRY DATA (anything can put geometry in the list)
    if RENDER_LAYERS['debug']:
        for type, geo in N.debug_geometry:
            if type == 'curve':
                draw_curve(geo)
            elif type == 'dot':
                #print(geo)
                draw_dot(geo)

    # DRAW ROUTE (debug)
    for elem in route_geo:
        draw_curve(elem, '#ff8000', width=3)
    #for elem, _ in route:
    #    
    #    draw_curve(network.roads[elem].curve, '#ff8000', width=3)

    # DRAW VEHICLES
    if RENDER_LAYERS['vehicles']:
        for v in network.vehicles:
            #print(v.path_following)
            if v.pos != (None, None):
                if on_screen(v.pos):
                    vec_col = '#ff00ff'
                    
                    if v.path_following == 'node':
                        vec_col = '#ffff00'
                    
                    vec_size = v.acceleration * 5

                    draw_vec(v.pos, v.dir, vec_size, vec_col)

                    # draw speed targets
                    if RENDER_LAYERS['speed_tgts']:
                        for tgt in v.tgt_speed_graph:
                            if tgt[1] > 0:
                                draw_cross(move_pt(v.pos, v.dir, tgt[1]), 1, TGT_COL[tgt[0]])

                        draw_dot(move_pt(v.pos, v.dir, -v.distance_along_path))
    
    # DRAW MARKERS
    for marker in network.markers:

        road = network.roads[marker.road_index]

        path_eval = road.curve.eval(marker.t)

        pos = offset_pt(path_eval[0], path_eval[1], 0)
        
        draw_dot(pos, 8)


    # INTERACTABLE ELEMENTS
    if RENDER_LAYERS['hovered_content']:
        
        if hovered_content[0] is None:
            last_log = None # hover over nothing should reset
        else:
            if last_log != hovered_content:
                #print(hovered_content)
                last_log = hovered_content
            
            # render
            if hovered_content[0] == 'road':
                draw_curve(network.roads[hovered_content[1]].curve, '#ffffff', width=5)

            elif hovered_content[0] == 'node':
                draw_dot(network.nodes[hovered_content[1]].pos(), 6, '#ffffff')

            elif hovered_content[0] == 'connection':
                pass
            


    # UPDATE SCREEN
    pygame.display.flip()
    dt = clock.tick(60) / 1000 # delta time
    simulation_deviation += dt

pygame.quit()