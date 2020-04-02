import numpy as np
from rrt_utils.search_space.search_space import SearchSpace
from rrt_utils.rrt.rrt_star import RRTStar
from rrt_utils.utilities.plotting import Plot
from receding_h_utils import create_obstacles
VOXEL_SIZE = 1
VOXMAP_CUBE_SIZE = 200

class Drone():
    def __init__(self):
        self.north_offset = -316
        self.east_offset = -445
        self.local_position = np.array([0, 0, -70])
        self.grid_start = (316, 445)
        self.grid_goal = (370, 431)

self = Drone()

# NED
vehicle_pos_in_grid = np.array([self.local_position[0] - self.north_offset, self.local_position[1] - self.east_offset, self.local_position[2]])

data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
Obstacles, X_dimensions = create_obstacles(data, voxel_size=VOXEL_SIZE, cube_size=VOXMAP_CUBE_SIZE, vehicle_pos=vehicle_pos_in_grid)

x_init = ((vehicle_pos_in_grid[1]) // VOXEL_SIZE, (vehicle_pos_in_grid[0]) // VOXEL_SIZE, abs(vehicle_pos_in_grid[2]) // VOXEL_SIZE)  # starting location
x_goal = (self.grid_goal[1] // VOXEL_SIZE, self.grid_goal[0] // VOXEL_SIZE, (abs(self.local_position[2]) + 20) // VOXEL_SIZE)  # goal location

Q = np.array([(8, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
rewire_count = 8  # optional, number of nearby branches to rewire
prc = 0.1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
path = rrt.rrt_star()
# plot
plot = Plot("rrt_star_3d")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)