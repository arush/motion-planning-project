import numpy as np

def create_obstacles(data=None, voxel_size=5, cube_size=100, vehicle_pos=np.array([315,444,-5])):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data. along with dimensions of the window cut by the cube_size
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    The `cube_size` argument sets the volume of the search space.
    The `vehicle_pos` argument is the vehicle's position in data's coordinate space NED
        used to calc the cube's edges or "window" as it moves through the map
    """
    if data is None:
        raise "Must provide data"

    # resize location into voxel_size coords
    vehicle_pos_in_voxel_size = vehicle_pos // voxel_size
    print('vehicle in vox size: {}'.format(vehicle_pos_in_voxel_size))
    # assume size of the map
    north_size = cube_size
    east_size = cube_size
    alt_size = cube_size
 
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    print(alt_max)
    
    
    # offset the data grid to start at 0,0
    offset_grid_east_min = 0
    offset_grid_north_min = 0
    offset_grid_z_min = 0

    offset_grid_east_max = east_max - east_min
    offset_grid_north_max = north_max - north_min
    offset_grid_z_max = alt_max
    
    print('offset_grid_east_min: {}\noffset_grid_north_min: {}\noffset_grid_z_min: {}\noffset_grid_east_max: {}\noffset_grid_north_max: {}\noffset_grid_z_max: {}\n'.format(offset_grid_east_min,offset_grid_north_min,offset_grid_z_min,offset_grid_east_max,offset_grid_north_max,offset_grid_z_max))

    # calc moving window
    # y is north, x is east
    if vehicle_pos_in_voxel_size[0] < cube_size // 2:
        window_y_min = 0
        window_y_max = int(cube_size)
    elif vehicle_pos_in_voxel_size[0] >= (offset_grid_east_max // voxel_size - cube_size):
        window_y_min = int(offset_grid_north_max // voxel_size - cube_size)
        window_y_max = int(offset_grid_north_max // voxel_size)
    else:
        window_y_min = vehicle_pos_in_voxel_size[0] - cube_size // 2
        window_y_max = vehicle_pos_in_voxel_size[0] + cube_size // 2
    
    if vehicle_pos_in_voxel_size[1] < cube_size // 2:
        window_x_min = 0
        window_x_max = int(cube_size)
    elif vehicle_pos_in_voxel_size[1] >= (offset_grid_north_max // voxel_size - cube_size):
        window_x_min = int(offset_grid_east_max // voxel_size - cube_size)
        window_x_max = int(offset_grid_east_max // voxel_size)
    else:
        window_x_min = int(vehicle_pos_in_voxel_size[1] - cube_size // 2)
        window_x_max = int(vehicle_pos_in_voxel_size[1] + cube_size // 2)

    if vehicle_pos_in_voxel_size[2] < cube_size // 2:
        window_alt_min = 0
        window_alt_max = int(cube_size)
    elif vehicle_pos_in_voxel_size[2] >= (alt_max // voxel_size - cube_size):
        window_alt_min = int(alt_max // voxel_size - cube_size)
        window_alt_max = int(alt_max // voxel_size)
    else:
        window_alt_min = int(vehicle_pos_in_voxel_size[2] - cube_size // 2)
        window_alt_max = int(vehicle_pos_in_voxel_size[2] + cube_size // 2)

    print('window_y_min: {}\nwindow_x_min: {}\nwindow_alt_min: {}\nwindow_y_max: {}\nwindow_x_max: {}\nwindow_alt_max: {}\n'.format(window_y_min,window_x_min,window_alt_min,window_y_max,window_x_max,window_alt_max))
    
    # Dimensions of Search Space
    X_dimensions = np.array([
        (window_x_min, window_x_max), # x
        (window_y_min, window_y_max), # y
        (window_alt_min, window_alt_max)  # z
    ])

    # Create an empty grid
    obstacles = np.array([0, 0, 0, 1, 1, 1])

    for i in range(data.shape[0]):
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
        ]
        
        height = int(alt + d_alt) // voxel_size

        x_lower = obstacle[2]
        y_lower = obstacle[0]
        z_lower = 0
        x_upper = obstacle[3]
        y_upper = obstacle[1]
        z_upper = height
        

        # remove blocks outside the window
        if x_upper < window_x_min or y_upper < window_y_min:
            continue

        if x_lower > east_size + window_x_min or y_lower > north_size + window_y_min:
            continue

        # obstacles
        # (x_lower, y_lower, z_lower, x_upper, y_upper, z_upper)
        row = np.array([x_lower, y_lower, 0, x_upper, y_upper, height])
        obstacles = np.vstack((obstacles, row))

    try:
        if obstacles.shape[1] == 6:
            obstacles = np.delete(obstacles, 0, 0)
            print('built {} obstacles'.format(obstacles.shape[0]))
    except IndexError:
        print('No obstacles created so sticking with the placeholder')
    
    print(obstacles)
    return (obstacles, X_dimensions)