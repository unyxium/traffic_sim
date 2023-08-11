# A Vehicular Traffic Simulation & Analysis Solution For Urban Project Planning Within Grasshopper 

A Python-based toolkit for Grasshopper to perform microscopic multi-agent vehicle simulation through a set of Grasshopper components (including Hops). It generates and outputs data for analysis and visualisation including recorded vehicle positions, road congestion highlighting, and routes. The road network consists of inputted curve geometry and attributes such as lane configurations, intersections, and vehicle trips. Vehicles are able to navigate to and from points of interests assigned by the user and perform their own pathfinding, collision avoidance, and following of intersection signalling.

## Dependencies
- ghhops_server
- flask
- pygame (optional, if you want to run the Pygame tool debugger)

## Setup
The code for the Hops components must be run on a server.

To locally host, run `hops_server.py`, which will start a Flask server on your computer. Now open Rhino and Grasshopper. Create a Hops component and enter the address. The root address will provide a component listing all the components available. 