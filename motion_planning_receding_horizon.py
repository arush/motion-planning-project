import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from receding_h_utils import create_obstacles
from rrt_utils.search_space.search_space import SearchSpace
from rrt_utils.rrt.rrt_star import RRTStar
from rrt_utils.utilities.plotting import Plot

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

VOXEL_SIZE = 1
VOXMAP_CUBE_SIZE = 300
# Set goal as some arbitrary position on the grid
# TODO: adapt to set goal as latitude / longitude position and convert
GOAL_LAT = 37.792968
GOAL_LON = -122.397600

TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

# really far
# GOAL_LAT = 37.7969561
# GOAL_LON = -122.3991139

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    REPLAN = auto()
    LOITER = auto()
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

        # for replanning
        self.global_goal_pos = None
        self.local_goal_pos = None
        self.replan_in_progress = False
        self.grid_start = None
        self.grid_goal = None
        self.global_plan = None
        self.north_offset = None
        self.east_offset = None

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
    

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.replan_transition()
        elif self.flight_state == States.WAYPOINT:
            # since there will be a minimum of 1 waypoint if we haven't reached destination
            # when there is only 1 waypoint we need to replan 
            if (np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 2.0) and (abs(self.local_position[2]) >= 0.95 * abs(self.target_position[2])):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                elif len(self.waypoints) == 0 and not self.goal_reached():
                    self.replan_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
    
    def goal_reached(self):
        if self.local_goal_pos is not None:
            return (np.linalg.norm(self.local_goal_pos[0:2] - self.local_position[0:2]) < 1.0) and (abs(self.local_position[2]) >= 0.95 * abs(self.local_goal_pos[2]))
        else:
            return False

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
                    self.plan_global_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.LOITER:
                print('state LOITER',len(self.waypoints))
                self.replan_transition()
            elif self.flight_state == States.REPLAN:
                # if no plan exists yet
                print('state REPLAN {} goal reached? {}'.format(len(self.waypoints), self.goal_reached()))
                if len(self.waypoints) == 0 and not self.goal_reached():
                    self.loiter_transition()
                elif len(self.waypoints) > 0 and not self.goal_reached():
                    self.waypoint_transition()
                else:
                    print('goal reached? ', self.goal_reached())
                    
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

    # loiter state is needed for a continuous event based loop between loiter, replan and waypoint
    def loiter_transition(self):
        self.flight_state = States.LOITER
        print("loiter transition")

    def replan_transition(self):
        # we have grid_start and grid_goal
        # and for fun lets elevate the goal 10m on every replanning iteration
        # so we get a nice 3d path

        self.flight_state = States.REPLAN
        print("hover replan transition")

        if self.replan_in_progress:
            print('replan already in progress, moving to loiter')
            pass # state will automatically transition to loiter
        else: 
            self.replan_in_progress = True

        # pick the last point of the path within the cube
        vehicle_pos_in_grid = np.array([self.local_position[0] - self.north_offset, self.local_position[1] - self.east_offset, self.local_position[2]])
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        Obstacles, X_dimensions = create_obstacles(data, voxel_size=VOXEL_SIZE, cube_size=VOXMAP_CUBE_SIZE, vehicle_pos=vehicle_pos_in_grid)

        # convert NED to x, y
        # remember x is East, y is North
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

        waypoints = [[int(p[1] * VOXEL_SIZE + self.north_offset), int(p[0] * VOXEL_SIZE + self.east_offset), int(p[2] * VOXEL_SIZE), 0] for p in path]
        print(waypoints)
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
        self.replan_in_progress = False
        # plot
        # plot = Plot("rrt_star_3d")
        # plot.plot_tree(X, rrt.trees)
        # if path is not None:
        #     plot.plot_path(X, path)
        # plot.plot_obstacles(X, Obstacles)
        # plot.plot_start(X, x_init)
        # plot.plot_goal(X, x_goal)
        # plot.draw(auto_open=True)

        
        

    def plan_global_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        [lat0, lon0] = np.loadtxt('colliders.csv', 
                            delimiter=',', 
                            dtype='Float64', 
                            converters={0: lambda s: float(s.strip('lat0 '.encode())), 
                                        1: lambda s: float(s.strip('lon0 '.encode()))}, 
                            max_rows=1)
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(longitude=lon0, latitude=lat0, altitude=0)

        # can't see why this is needed if we already have self.local_position
        # TODO: convert to current local position using global_to_local()
        current_local_pos = global_to_local(global_position=self.global_position, global_home=self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        print('local position {0}\nself.local_position {1}'.format(current_local_pos, self.local_position))

        
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, self.north_offset, self.east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(self.north_offset, self.east_offset))

        # Define starting point on the grid
        # TODO: convert start position to current position rather than map center

        self.grid_start = (int(current_local_pos[0] - self.north_offset), int(current_local_pos[1] - self.east_offset))
        # starting TARGET_ALTITUDE meters above current local_position

        self.global_goal_pos = np.array([GOAL_LON, GOAL_LAT, TARGET_ALTITUDE])

        self.local_goal_pos = np.array([*global_to_local(global_position=self.global_goal_pos, global_home=self.global_home)])
        self.grid_goal = (int(self.local_goal_pos[0] - self.north_offset), int(self.local_goal_pos[1] - self.east_offset))

        print('Local Start and Goal (NED): ', self.grid_start, self.grid_goal)
        self.global_plan, _ = a_star(grid, heuristic, self.grid_start, self.grid_goal)

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # pruned_path = prune_path(detailed_path)

        # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        # Set self.waypoints

        

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
