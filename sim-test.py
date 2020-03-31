import numpy as np
import time
from udacidrone.frame_utils import global_to_local

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection

conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True, PX4=False)
time.sleep(2)
drone = Drone(conn)
time.sleep(2)
drone.start()
time.sleep(2)
drone.take_control()
time.sleep(2)
drone.arm()


local position [ 6.97714961e+01 -1.32430816e+01  6.60000000e-02]
self.local_position [ 6.99566727e+01 -1.29187117e+01  6.64269775e-02]

local position [ 6.97714961e+01 -1.32430816e+01  6.60000000e-02]
self.local_position [ 6.99566727e+01 -1.29187117e+01  6.64269775e-02]
