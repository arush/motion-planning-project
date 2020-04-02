## 3D Motion Planning with 2.5D A* and 3D RRT* with Receding Horizon

Full demo: https://www.youtube.com/watch?v=VtXHbCOOd1I&feature=youtu.be

[![youtube: Receding Horizon RRT* 3D Motion Planning](E8mRNl.gif)](https://www.youtube.com/watch?v=VtXHbCOOd1I&feature=youtu.be "Receding Horizon RRT* 3D Motion Planning")



---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

I completed this project twice. Once with a 2.5D grid plan with A* and another version by adding a receding horizon 3d plan with RRT*.

The solutions are:
[motion_planning_grid.py](motion_planning_grid.py)

[motion_planning_receding_horizon.py](motion_planning_receding_horizon.py)


### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

These scripts contain a basic planning implementation based on the `backyard_flyer.py` code. 

##### Difference between last project
The main modification is there is an additional `PLANNING` state in which the `MavlinkPlanning` instance executes a planning strategy `self.plan_path` before executing the strategy by converting the result of the planning to waypoints. 

A visualization of the waypoints in the sim is displayed by a new method called `send_waypoints` that sends the waypoints over the mavlink connection in the MessagePack binary serialization format. 

The `state_callback` is called when the drone is set to the `PLANNING` state and since this python code is all syncronously executed, we know it got to this state by already executing the `self.plan_path` and `self.send_waypoints` methods, so we can safely takeoff. The drone then follows the takeoff, waypoints, landing and disarming transition in the same way the backyard flyer did.

This is a picture of the path it creates
![Top Down View](https://i.imgur.com/1kmtspv.png)

##### So how does the `plan_path` method work? 
First it loads the `colliders.csv` and skips the first two rows and loads the obstacle data into memory. 

We make use of the `create_grid` method to generate a basic 2 dimensional array marking every location between the obstacles' top, bottom, left and right sides as `1` instead of `0`. There is no obstacle height in this map, although `TARGET_ALTITUDE` is used in the `create_grid` method to detect if an obstacle should exist in that plane or not.

It sets the start as the map center and the goal as 10m north and 10m east of the start.

It uses the A* algorithim from `planning_utils` and the defined `heuristic` function to generate a path through the grid from start to goal.

It runs through each point in the path, adds the north and east offset to translate between grid coords and the global frame, uses the `TARGET_ALTITUDE` from before and keeps a zero heading throughout.

<!-- 
Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd
 -->


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
I extracted the first 2 lines of the csv with the `np.loadtxt` since it didn't require importing a new package like `csv` or something. I used 2 lambda functions to strip the columns and extract the floating point values for lat lon. Then I used the built in method `set_home_position` with a zero altitude.


#### 2. Set your current local position
I use the local postion from `current_local_position` instead of `self.local_position` because the former is more accurate. I can't understand why.

#### 3. Set grid start position from local position
I'm using the `current_local_position` subtracting the offsets and casting to `int` to create coords for the `grid_start`.

#### 4. Set grid goal position from geodetic coords
Using arbitrary lat lon as the goal by converting to local coords then adjusting with the offset to put it into the grid coords.

![Arbitrary Lat Lon Goal](https://i.imgur.com/IdH7VFt.png)

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I modified `a_star` to include `NORTH_EAST NORTH_WEST SOUTH_EAST SOUTH_WEST` actions by summing the coordinates and adding a diagonal cost of `math.sqrt(2)`. 

I also modified `valid_actions` to incorporate potential off-grid movements and obstacles in the diagonal space. The simulator now moves in diagonals:

![Diagonal movement](https://i.imgur.com/pUav8la.png)


#### 6. Cull waypoints 
To cull waypoints I chose the collinearity test. I created three functions in the `planning_utils` file

The collinearity method converts 2D points to a 3 dimensional array with the z value as 1. This allows us to use the determinant method. If the determinant of a matrix with 3 points is zero then they are deemed collinear.

The `point` function simply converts a 2D coordinate to a 3 dimensional array element.

The `collinearity_check` takes 3 points, puts them in a matrix and checks whether the determinant is near enough zero.

The `prune_path` method loops through the path in sets of 3 points at a time and removes the middle of the 3 points if the collinearity check is truthy.


### Execute the flight
#### 1. Does it work?

Here is a photo of the drone executing the plan, rounding buildings with culled waypoints.

![Works](https://i.imgur.com/58CLCQA.png)


### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For the extra challenge see the video at [https://www.youtube.com/watch?v=VtXHbCOOd1I&feature=youtu.be](https://www.youtube.com/watch?v=VtXHbCOOd1I&feature=youtu.be)

and the corresponding code at [motion_planning_receding_horizon.py](motion_planning_receding_horizon.py)

It computes a global coarse plan over a low fidelity 2.5D map with A* then constantly replans as the vehicle flies. This constant replanning provides increased safety and redundancy in case of incorrect coarse map data or anything that might appear along the way like wind.

Once the vehicle takes off it begins a hi-fidelity 3d dense map plan of it's immediate surroundings, a 100m^3 cube. This is configurable based on on-board compute power. It then uses RRT* to find a 3d path through to the next waypoint. 

On every replan it produces an HTML Plotly visualization of what obstacles it sees inside the 100m^3 "window" and the resultant path of the 3D RRT* algorithm.

Once it reaches the coarse plan goal it performs one last receding horizon replan to find a goal 10m above and then lands.


