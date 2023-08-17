# This file is the hops server running in flask

from flask import Flask
import ghhops_server as hs
import json
import network as N
import math
import time

app = Flask(__name__)
hops = hs.Hops(app)


CATEGORY = 'Extra'
DEFAULT_ICON = 'icon_graph.png'


@hops.component(
    '/',
    name='Info',
    description='Traffic simulation and analysis toolkit.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[],
    outputs=[
        hs.HopsString('Details', 'D', 'Details'),
        hs.HopsString('Components', 'C', 'List of components available (use these in the address)')
    ]
)
def component_info():
    """Returns info about the hops server."""
    return 'Created by Martin Ibbett, 2023', ['trim', 'simulation', 'pathfind', 'fillets', 'add_random_traffic', 'add_vehicles', 'add_markers', 'mapping_curves']



@hops.component(
    '/trim',
    name='Trim',
    description='Trim a road network (shorten the road edges where they meet at intersections so there is no overlap).',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
        hs.HopsNumber('Corner radius', 'Radius', 'Radius of intersection corners', optional=True)
    ],
    outputs=[
        hs.HopsString('Network', 'Network', 'Trimmed JSON network')
    ]
)
def component_trim(json_network, corner_radius=2):
    json_network = json.loads(json_network)

    network = N.load_packed_network(json_network)

    if 'connection_cache' not in json_network['info']['properties']:
        network.generate_connection_cache()
    
    network.trim_roads(15, corner_radius, 4)

    return network.data()



@hops.component(
    '/fillets',
    name='Fillets',
    description='Get fillet curves around intersections.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
    ],
    outputs=[
        hs.HopsString('Fillets', 'Fillets', 'Fillet curves as JSON'),
    ]
)
def component_fillets(json_network):
    json_network = json.loads(json_network)

    network = N.load_packed_network(json_network)

    if 'connection_cache' not in json_network['info']['properties']:
        network.generate_connection_cache()
    
    return network.get_fillets(True)



@hops.component(
    '/random_traffic',
    name='RandomTraffic',
    description='Add random traffic to the road network.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
        hs.HopsInteger('Count', 'Count', 'Number of vehicles to spawn'),
        hs.HopsInteger('Seed', 'Seed', 'Seed value for randomness', optional=True),
    ],
    outputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
    ]
)
def component_random_traffic(json_network, count, seed=0):
    # Note that there is no generation of lane mapping yet, the simulator does it
    N.set_seed(seed)
    
    network = N.load_packed_network(json.loads(json_network))
    
    if not isinstance(count, int) or count < 0: # if count is invalid, add vehicles to 1/5 of roads
        count = math.ceil(len(network.roads) / 5)
    
    network.add_random_traffic(count)
    
    return network.data()



@hops.component(
    '/add_vehicles',
    name='AddVehicles',
    description='Add vehicles to the road network. Note that a simpler RandomTraffic exists if you just need randomly distributed traffic on the network.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
        hs.HopsString('Vehicles', 'Vehicles', 'JSON vehicle data as a list', access=hs.HopsParamAccess.LIST, optional=True),
    ],
    outputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
    ]
)
def component_add_vehicles(json_network, vehicles=''):
    # Note that there is no generation of lane mapping yet
    network = N.load_packed_network(json.loads(json_network))
    
    if 'connection_cache' not in json_network['info']['properties']:
        network.generate_connection_cache()

    if 'lane_mapping' not in json_network['info']['properties']:
        network.generate_lane_mapping()
    
    for vehicle in vehicles:
        if not vehicle == '':
            vehicle: dict
            network.add_vehicle(
                vehicle['road_index'], 
                vehicle.get('lane_index', 0), 
                vehicle.get('t', 0), 
                vehicle.get('route', None),
                vehicle.get('speed', None),
            ) 

    return network.data()



@hops.component(
    '/add_markers',
    name='AddMarkers',
    description='Add markers to the road network.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
        hs.HopsString('Markers', 'Markers', 'JSON markers data as a list', access=hs.HopsParamAccess.LIST, optional=True),
    ],
    outputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
    ]
)
def component_add_markers(json_network, markers=''):
    json_network = json.loads(json_network)
    network = N.load_packed_network(json_network)
    
    if 'connection_cache' not in json_network['info']['properties']:
        network.generate_connection_cache()

    if 'lane_mapping' not in json_network['info']['properties']:
        network.generate_lane_mapping()
    
    for marker in markers:
        if not marker == '':
            
            marker: dict = json.loads(marker)
            
            if marker['road_index'] >= len(network.roads):
                raise Exception('Marker road index out of range')

            if marker['type'] == 'spawner_pathfind' or marker['type'] == 'spawner_pathfind' or marker['type'] == 'spawner':
                network.add_spawner(
                    marker['road_index'], 
                    marker.get('lane_indices', [0]),
                    marker.get('t', 0),
                    marker.get('spacing', 10),
                    marker.get('spawns_remaining', 100),
                    marker.get('target', None),
                    )

    return network.data()


@hops.component(
    '/mapping_curves',
    name='MappingCrvs',
    description='Generate all mapping curves as preview.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
    ],
    outputs=[
        hs.HopsString('Curves', 'Crvs', 'Mapping curves'),
    ]
)
def component_mapping_curves(json_network):
    json_network = json.loads(json_network)
    network = N.load_packed_network(json_network)

    if 'connection_cache' not in json_network['info']['properties']:
        network.generate_connection_cache()

    if 'lane_mapping' not in json_network['info']['properties']:
        network.generate_lane_mapping()

    if 'trim' not in json_network['info']['properties']:
        network.trim_roads()
    
    network.generate_mapping_curves()

    output_curves = []
    for node_index, node in enumerate(network.nodes):
        oc = []
        for crv in node.mapping_curves.values():
            oc.append(crv.data())
        output_curves.append(oc)
    
    return output_curves



@hops.component(
    '/simulation',
    name='Simulation',
    description='Run a vehicle simulation on a given JSON network.',
    category=CATEGORY,
    icon=DEFAULT_ICON,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
        hs.HopsString('Simulation parameters', 'Parameters', 'JSON file containing the parameters to start with'),
        hs.HopsInteger('Seed', 'Seed', 'Seed value for randomness.', optional=True),
    ],
    outputs=[
        hs.HopsString('Statistics', 'Stats', 'Overall statistics, JSON data'),
        hs.HopsString('Animation', 'Animation', 'Vehicle animation, JSON data'),
        hs.HopsString('Analysis', 'Analysis', 'Combination of per-road analyses, JSON data'),
    ]
)
def component_simulation(json_network, params, seed=0):
    N.set_seed(seed)

    stats = {}

    json_network = json.loads(json_network)
    # LOAD PARAMS
    parameters = json.loads(params)
    duration = parameters.get('duration', 20)
    timestep = parameters.get('timestep', 1/20)
    save_skip = parameters.get('save_skip', 5)
    timeout = parameters.get('timeout', 30)

    # LOAD NETWORK
    network = N.load_packed_network(json_network)
    network.record = True
    network.record_duration = duration

    if 'connection_cache' not in json_network['info']['properties']:
        network.generate_connection_cache()

    if 'lane_mapping' not in json_network['info']['properties']:
        network.generate_lane_mapping()

    if 'trim' not in json_network['info']['properties']:
        network.trim_roads()

    network.generate_mapping_curves() # does not get cached
    network.generate_signalling()

    print(network) # info

    # SIMULATE
    timeout_time = time.time() + min(timeout, 3600) # absolute maximum timeout of 1 hr
    total_frames = int(network.record_duration // timestep) # number of simulation frames

    for frame_i in range(total_frames):
        network.update_markers(timestep)
        network.update_vehicles(timestep)
        network.analysis_frame(timestep)

        if frame_i % save_skip == 0:
            network.save_data_frame() # save
        network.time += timestep

        if time.time() > timeout_time: # timeout
            print(time.time(), timeout_time)
            stats['timeout'] = f'Maximum time reached, could not complete fully. {frame_i}/{total_frames}'
            print('COULD NOT COMPLETE - TIMEOUT')
            break # allows for saving

    # SAVE
    anim = {
        'duration_sec':duration,
        'simulation_timestep':timestep,
        'animation_timestep':timestep*save_skip,
        'save_skip':save_skip,
        'vehicles':network.get_animation()
    }
    
    return stats, anim, network.get_analysis()


@hops.component(
    '/pathfind',
    name='Pathfind',
    description='Find the shortest path from a start road to an end road. If no end is given, default to random walk.',
    icon=DEFAULT_ICON,
    category=CATEGORY,
    inputs=[
        hs.HopsString('Network', 'Network', 'JSON network'),
        hs.HopsInteger('Start road index', 'Start road', 'Start road index'),
        hs.HopsInteger('Start lane index', 'Start lane', 'Start lane index'),
        hs.HopsInteger('End road index', 'End road', 'End road index', optional=True),
        hs.HopsInteger('End lane index', 'End lane', 'End lane index', optional=True),
        hs.HopsInteger('Depth limit', 'Depth limit', 'Depth limit, the number of iterations allowed before failing', optional=True),
    ],
    outputs=[
        hs.HopsString('Route', 'Route', 'Route data as JSON'),
    ]
)
def component_pathfind(json_network, start_road, start_lane, end_road=-1, end_lane=-1, depth_limit=-1):
    json_network = json.loads(json_network)
    
    network = N.load_packed_network(json_network)
    network.generate_connection_cache()
    network.generate_lane_mapping()

    if depth_limit < 0:
        depth_limit = None
    if end_road < 0:
        end_road = None
    if end_lane < 0:
        end_lane = None
    
    route = network.create_vehicle_route((start_road, start_lane), (end_road, end_lane), depth_limit)
    
    return [route]


# http://127.0.0.1:5000
if __name__ == '__main__':
    app.run(debug=True)
