import argparse
import planning_utils
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global

import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx


nx.__version__ # should be 2.1


import numpy as np
import matplotlib.pyplot as plt
from sampling import Sampler
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 10

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', 'r') as terrainData:
            fields = terrainData.read().strip().split(",")
            lat0 = float(fields[0].split()[1])
            lon0 = float(fields[1].split()[1])
        
        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)

        # Convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # Convert start position to current position rather than map center
        grid_start = (int(local_position[0] - north_offset), int(local_position[1] - east_offset))        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10 )
        # Adapt to set goal as latitude / longitude position and convert
        grid_goal_pos = (grid_goal[0], grid_goal[0], TARGET_ALTITUDE)
        global_goal = local_to_global(grid_goal_pos, self.global_home)
        local_goal = global_to_local(global_goal, self.global_home)

        # Run A* to find a path from start to goal
        # Add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        #  path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        
        # Prune path to minimize number of waypoints
        pruned_path = prune_path(path)
        print('{0} paths pruned to {1}\n'.format(len(path), len(pruned_path)))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        print(waypoints)
        print()

        # Set self.waypoints
        self.waypoints = waypoints
        
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.connection.start()
        self.send_waypoints()

        # ========================================================================================
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # Using the Probablistic approach
        # ========================================================================================
        
        plt.rcParams['figure.figsize'] = 15, 15

        filename = 'colliders.csv'
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
        print(data)

        from sampling import Sampler
        sampler = Sampler(data)
        polygons = sampler._polygons
        nodes = sampler.sample(800)
        print(len(nodes))
        nodes[0]= (local_goal[0],local_goal[1],local_goal[2])

        import numpy.linalg as LA
        from sklearn.neighbors import KDTree

        def can_connect(n1, n2):
            l = LineString([n1, n2])
            for p in polygons:
                if p.crosses(l) and p.height >= min(n1[2], n2[2]):
                    return False
            return True

        def create_graph(nodes, k):
            g = nx.Graph()
            tree = KDTree(nodes)
            for n1 in nodes:
                # for each node connect try to connect to k nearest nodes
                idxs = tree.query([n1], k, return_distance=False)[0]
        
                for idx in idxs:
                    n2 = nodes[idx]
                    if n2 == n1:
                        continue
                
                    if can_connect(n1, n2):
                        g.add_edge(n1, n2, weight=1)
            return g

        import time

        t0 = time.time()
        g = create_graph(nodes, 10)
        print('graph took {0} seconds to build'.format(time.time()-t0))
        print("Number of edges", len(g.edges))

        from grid import create_grid_prob
        grid = create_grid_prob(data, sampler._zmax, 1)

        fig = plt.figure()

        plt.imshow(grid, cmap='Greys', origin='lower')

        nmin = np.min(data[:, 0])
        emin = np.min(data[:, 1])

        # draw edges
        for (n1, n2) in g.edges:
            plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black' , alpha=0.5)

        # draw all nodes
        for n1 in nodes:
            plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')
    
        # draw connected nodes
        for n1 in g.nodes:
            plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    
        plt.xlabel('NORTH')
        plt.ylabel('EAST')

        plt.show()

        def heuristic_prob(n1, n2):
            # TODO: finish
            return LA.norm(np.array(n2) - np.array(n1))

        def a_star_prob(graph, heuristic_prob, start, goal):
            """Modified A* to work with NetworkX graphs."""
    
            path = []
            queue = PriorityQueue()
            queue.put((0, start))
            visited = set(start)

            branch = {}
            found = False
    
            while not queue.empty():
                item = queue.get()
                current_cost = item[0]
                current_node = item[1]

                if current_node == goal:        
                    print('Found a path.')
                    found = True
                    break
                else:
                    for next_node in graph[current_node]:
                        cost = graph.edges[current_node, next_node]['weight']
                        new_cost = current_cost + cost + heuristic(next_node, goal)
                
                        if next_node not in visited:                
                            visited.add(next_node)               
                            queue.put((new_cost, next_node))
                    
                            branch[next_node] = (new_cost, current_node)
             
            path = []
            path_cost = 0
            if found:
        
                # retrace steps
                path = []
                n = goal
                path_cost = branch[n][0]
                while branch[n][1] != start:
                    path.append(branch[n][1])
                    n = branch[n][1]
                path.append(branch[n][1])
            
            return path[::-1], path_cost


        start = list(g.nodes)[0]
        k = np.random.randint(len(g.nodes))
        print(k, len(g.nodes))
        k=len(g.nodes) - 1
        goal = list(g.nodes)[k]

        print("\n\n")
        print("Start at : {0}",start)
        print("\n")
        print("Goal at : {0}" ,goal)
        print("\n\n")

        path_prob, cost = a_star_prob(g, heuristic, start, goal)
        
        print(len(path), path_prob)

        path_pairs = zip(path_prob[:-1], path_prob[1:])
        for (n1, n2) in path_pairs:
            print("\n\t\t")
            print(n1, n2)

        fig = plt.figure()

        plt.imshow(grid, cmap='Greys', origin='lower')

        nmin = np.min(data[:, 0])
        emin = np.min(data[:, 1])

        # draw nodes
        for n1 in g.nodes:
            plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
    
        # draw edges
        for (n1, n2) in g.edges:
            plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'gray')
    
        # TODO: add code to visualize the path
        path_pairs = zip(path_prob[:-1], path_prob[1:])
        for (n1, n2) in path_pairs:
            plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green',linewidth=5.0)


        plt.xlabel('NORTH')
        plt.ylabel('EAST')

        plt.show()

        # ========================================================================================

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
