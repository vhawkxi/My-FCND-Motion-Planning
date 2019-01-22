# Project: 3D Motion Planning

![Quad Movie](./misc/builtin2.mp4)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

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