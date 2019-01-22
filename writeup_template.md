# Project: 3D Motion Planning

![Quad Movie](https://github.com/vhawkxi/My-FCND-Motion-Planning/blob/master/misc/builtin2.mp4)

---
## Starter Code

Here I will explain the starter code. How does the code of backyard flyer differ in context from that of motion planning?

We start off by looking at:

``` Python
autoclass States(Enum):
        MANUAL = auto()
        ARMING = auto()
        TAKEOFF = auto()
        WAYPOINT = auto()
        LANDING = auto()
        DISARMING = auto()
        PLANNING = auto()auto
```

which adds the PLANNING state, a state that is invoked after the arming state has completed to calculate the way the drone is going to reach the destination.

``` Python
   def state_callback(self):
        if self.in_mission:
            ...  
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif ...
```

Next we look at the plan_path method:

``` Python
   def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 10

        self.target_position[2] = TARGET_ALTITUDE

        # ===========================================================
        # Point 1
        # ===========================================================
        # Read lat0, lon0 from colliders into floating point values
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

        # ===========================================================
        # Point 2
        # ===========================================================
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

        # ===========================================================
        # Point 3
        # ===========================================================
        # Run A* to find a path from start to goal
        # Add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # Prune path to minimize number of waypoints
        pruned_path = prune_path(path)
        print('{0} paths pruned to {1}\n'.format(len(path), len(pruned_path)))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        print(waypoints)
        print()
```

### Point 1

At point 1 we read the startig position from the _colliders.csv_ fie.  we then set the home position to this value.

### Point 2

At point 2 we define a grid for the environment the drone has to fly in, taking into consideration the safety space around the drone and the height we want to fly at.

### Point 3

At point 3 we use the A* method to find a path from the home to the end position, prune out unneccesarry points and then fly the path.

## Output

```auto
Logs\NavLog.txt
starting connection
arming transition
Searching for a path ...
global home [-122.39745   37.79248    0.     ], position [-1.22397449e+02  3.77924812e+01 -2.40000000e-02], local position [0.14283974 0.09875524 0.02475624]
North offset = -316, east offset = -445
Local Start and Goal:  (316, 445) (326, 455)
Found a path.
11 paths pruned to 11

[[0, 0, 5, 0], [1, 1, 5, 0], [2, 2, 5, 0], [3, 3, 5, 0], [4, 4, 5, 0], [5, 5, 5, 0], [6, 6, 5, 0], [7, 7, 5, 0], [8, 8, 5, 0], [9, 9, 5, 0], [10, 10, 5, 0]]

takeoff transition
waypoint transition
target position [0, 0, 5, 0]
waypoint transition
target position [1, 1, 5, 0]
waypoint transition
target position [2, 2, 5, 0]
waypoint transition
target position [3, 3, 5, 0]
waypoint transition
target position [4, 4, 5, 0]
waypoint transition
target position [5, 5, 5, 0]
waypoint transition
target position [6, 6, 5, 0]
waypoint transition
target position [7, 7, 5, 0]
waypoint transition
target position [8, 8, 5, 0]
waypoint transition
target position [9, 9, 5, 0]
waypoint transition
target position [10, 10, 5, 0]
landing transition
disarm transition
manual transition
Closing connection ...
Sending waypoints to simulator ...
```

---

## Different Approuch : Using the Staistcal Planning option

``` Python
        # ============================================================================
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # Using the Probablistic approach
        # ============================================================================

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

        # ============================================================================

```

### Output Statistical Option

``` Text  
[[-310.2389   -439.2315     85.5         5.          5.         85.5     ]
 [-300.2389   -439.2315     85.5         5.          5.         85.5     ]
 [-290.2389   -439.2315     85.5         5.          5.         85.5     ]
 ...
 [ 257.8061    425.1645      1.75852     1.292725    1.292725    1.944791]
 [ 293.9967    368.3391      3.557666    1.129456    1.129456    3.667319]
 [ 281.5162    354.4156      4.999351    1.053772    1.053772    4.950246]]
525
graph took 1079.7265810966492 seconds to build
Number of edges 1922
15 509

Start at : {0} (-124.98039657993408, 76.45325074348966, 5.0)

Goal at : {0} (297.8595123573992, 146.45758778791617, 5.0)

Found a path.
12 [(-124.98039657993408, 76.45325074348966, 5.0), (-73.39559966282371, 56.68042824184681, 5.0), (-2.0541043133638937, 23.405110131807817, 5.0), (58.17268260130396, 19.261048546552047, 5.0), (51.052775622925765, 78.28140373614599, 5.0), (107.24911169940839, 118.0870101435554, 5.0), (153.7479163937624, 105.50088754840795, 5.0), (223.67057912298498, 120.83859819860163, 5.0), (266.7072013303775, 121.42172936178144, 5.0), (316.48438487060736, 84.58816050481164, 5.0)]

(-124.98039657993408, 76.45325074348966, 5.0) (-73.39559966282371, 56.68042824184681, 5.0)

(-73.39559966282371, 56.68042824184681, 5.0) (-2.0541043133638937, 23.405110131807817, 5.0)

(-2.0541043133638937, 23.405110131807817, 5.0) (58.17268260130396, 19.261048546552047, 5.0)

(58.17268260130396, 19.261048546552047, 5.0) (51.052775622925765, 78.28140373614599, 5.0)

(51.052775622925765, 78.28140373614599, 5.0) (107.24911169940839, 118.0870101435554, 5.0)

(107.24911169940839, 118.0870101435554, 5.0) (153.7479163937624, 105.50088754840795, 5.0)

(153.7479163937624, 105.50088754840795, 5.0) (223.67057912298498, 120.83859819860163, 5.0)

(223.67057912298498, 120.83859819860163, 5.0) (266.7072013303775, 121.42172936178144, 5.0)

(266.7072013303775, 121.42172936178144, 5.0) (316.48438487060736, 84.58816050481164, 5.0)

```

### Images

![Edges](https://github.com/vhawkxi/My-FCND-Motion-Planning/blob/master/misc/Capture.PNG) ![Path](https://github.com/vhawkxi/My-FCND-Motion-Planning/blob/master/misc/Capture2.PNG)
