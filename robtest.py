import robkin
import numpy as np
import matplotlib.pyplot as plt
import warnings

def checkReach(robot_chain, tolerance, start_position, conversion, visualise, debug, graph):

    print(f"\nChecking reach for {robot_chain} \nwith tolerance {tolerance}meters and starting position {start_position}.")

    x0, y0, z0 = start_position
    end = abs(x0 * conversion)

    # Creates a 2D arrays of shape (end*2, end*2).
    # (This covers x from -end...(end-1), y from -end..(end-1).)
    robot_reach_bool = np.zeros((end*2, end*2), dtype=int)
    # Using dtype=object so IK solutions can be lists/arrays of varying length
    robot_reach_num  = np.empty((end*2, end*2), dtype=object)

    for i in range(-end, end):
        real_x = (i / conversion)

        for j in range(-end, end):
            real_y = j / conversion

            real_z = z0

            print(f"\n------------------------------------------------------------------------------------------------------"
                  f"\nChecking IK for [{real_x}, {real_y}, {real_z}]")
            ik_solutions = robkin.runAndCheckIK(robot_chain, [real_x, real_y, real_z],
                                                tolerance, debug=debug, graph=graph)
            # Offset array indices by +end since i,j can be negative
            arr_x = i + end
            arr_y = j + end

            if ik_solutions is None:
                robot_reach_bool[arr_x, arr_y] = 0
                print(f"Failed")
            else:
                robot_reach_bool[arr_x, arr_y] = 1
                robot_reach_num[arr_x, arr_y] = ik_solutions
                print(f"Success")
    if visualise:
        visualizeRobotReach(robot_reach_bool, end)

    return robot_reach_bool, robot_reach_num

def writeToFile(array, filename):
    print(f"Writing array to {filename}...")

    with open(filename, "w") as f:
        # Write header
        f.write(f"Matrix Name: {filename}\n")
        f.write(f"Format: [array_row, array_column] = [element_1, ..... element_n]\n")

        # Write array content
        for i, row in enumerate(array):
            for j, cell in enumerate(row):
                try:
                    # Check for null matrix or invalid data
                    if cell is None or (isinstance(cell, str) and "\0" in cell):
                        f.write(f"[{i}, {j}] = \n")  # Write row and column with empty data
                        # Removed print statements for logging
                        continue

                    # Safely convert cell to a list or string representation
                    if hasattr(cell, "tolist"):  # Check if the object has a tolist() method
                        cell_data = cell.tolist()
                    else:
                        cell_data = cell  # Use the object directly if tolist() is unavailable

                    # Write formatted line
                    formatted_line = f"[{i}, {j}] = {cell_data}\n"
                    f.write(formatted_line)

                except Exception:
                    # No error logging – just write a placeholder
                    f.write(f"[{i}, {j}] = \n")

    print(f"Finished writing array to {filename}.")


def visualizeRobotReach(robot_reach_bool, end):

    # The matrix shape is (2N, 2N) => 'end*2' in each dimension.
    # extent = [x_min, x_max, y_min, y_max]
    # We'll plot so that the bottom-left is (-end, -end)
    # and top-right is (+end, +end).

    extent = [-end, end, -end, end]

    plt.imshow(
        robot_reach_bool,
        cmap='gray',
        interpolation='nearest',
        extent=extent,
        origin='lower'  # so that -end is at the bottom, +end at the top
    )
    plt.colorbar(label='Value')
    plt.title("Visualization of Ones and Zeros Matrix")
    plt.xlabel("X Axis (from -end to +end)")
    plt.ylabel("Y Axis (from -end to +end)")
    plt.show()

def checkReach3D(
    robot_chain,
    tolerance,
    start_position,    # [x0, y0, z0]
    conversion,
    visualise,
    debug,
    graph
):
    """
    3D version of checkReach. Iterates over x, y, and z in a cube
    around (x0, y0, z0) to test whether the given robot can reach
    each coordinate.

    Args:
        robot_chain  : Robot configuration or chain ID
        tolerance    : Tolerance for IK solutions
        start_position : [x0, y0, z0]
        conversion   : Scaling factor from meters to array indices
        visualise    : Whether or not to visualize
        debug        : Print additional debug information
        graph        : Additional graph/plot object or utility

    Returns:
        robot_reach_bool_3d (3D numpy array of int) : 1 if reachable, else 0
        robot_reach_num_3d  (3D numpy array of object) : IK solutions
    """
    # Unpack start position
    x0, y0, z0 = start_position

    # Use absolute value * conversion for each dimension
    endX = int(abs(x0 * conversion))
    endY = int(abs(y0 * conversion))
    endZ = int(abs(z0 * conversion))

    print(f"\n[3D] Checking reach for {robot_chain} "
          f"with tolerance {tolerance} meters and starting position {start_position}")

    # Create 3D arrays for reach success/failure and for storing solutions
    robot_reach_bool_3d = np.zeros((endX * 2, endY * 2, endZ * 2), dtype=int)
    robot_reach_num_3d  = np.empty((endX * 2, endY * 2, endZ * 2), dtype=object)

    # Iterate over x, y, z in their respective ranges
    for i in range(-endX, endX):
        real_x = i / conversion

        for j in range(-endY, endY):
            real_y = j / conversion

            for k in range(-endZ, endZ, 10):
                real_z = k / conversion

                # Always print the position being checked
                print(f"\n------------------------------------------------------------------------------------------------------"
                      f"\n[3D] Checking IK for [{real_x}, {real_y}, {real_z}]")

                # Run IK
                ik_solutions = robkin.runAndCheckIK(
                    robot_chain,
                    [real_x, real_y, real_z],
                    tolerance,
                    debug=debug,
                    graph=graph
                )

                # Offset array indices by +end to make them 0-based
                arr_x = i + endX
                arr_y = j + endY
                arr_z = k + endZ

                # Store results and print success/failure
                if ik_solutions is None:
                    robot_reach_bool_3d[arr_x, arr_y, arr_z] = 0
                    print(f"Result: Failed")
                else:
                    robot_reach_bool_3d[arr_x, arr_y, arr_z] = 1
                    robot_reach_num_3d[arr_x, arr_y, arr_z]  = ik_solutions
                    print(f"Result: Success")

                # Additional debug information if debug is True
                if debug:
                    print(f"IK Solutions: {ik_solutions}")

    # Optional: visualize
    if visualise:
        visualizeRobotReach3D(robot_reach_bool_3d, endX, endY, endZ)

    return robot_reach_bool_3d, robot_reach_num_3d


def writeToFile3D(array_3d, filename):
    print(f"[3D] Writing 3D array to {filename}...")

    with open(filename, "w") as f:
        f.write(f"Matrix Name: {filename}\n")
        f.write(f"Format: [i, j, k] = [element_1, ... element_n]\n\n")

        dim_x, dim_y, dim_z = array_3d.shape
        for i in range(dim_x):
            for j in range(dim_y):
                for k in range(dim_z):
                    cell = array_3d[i, j, k]
                    try:
                        if cell is None or (isinstance(cell, str) and "\0" in cell):
                            f.write(f"[{i}, {j}, {k}] = \n")
                            continue

                        if hasattr(cell, "tolist"):
                            cell_data = cell.tolist()
                        else:
                            cell_data = cell

                        formatted_line = f"[{i}, {j}, {k}] = {cell_data}\n"
                        f.write(formatted_line)
                    except Exception:
                        f.write(f"[{i}, {j}, {k}] = \n")

    print(f"[3D] Finished writing 3D array to {filename}.")


def visualizeRobotReach3D(robot_reach_bool_3d, endX, endY, endZ):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D Visualization of Reachable Points (1's)")

    xs, ys, zs = [], [], []
    dim_x, dim_y, dim_z = robot_reach_bool_3d.shape

    for i in range(dim_x):
        for j in range(dim_y):
            for k in range(dim_z):
                if robot_reach_bool_3d[i, j, k] == 1:
                    xs.append(i - endX)
                    ys.append(j - endY)
                    zs.append(k - endZ)

    ax.scatter(xs, ys, zs, c='blue', marker='o', s=5)

    ax.set_xlabel("X axis (index offset)")
    ax.set_ylabel("Y axis (index offset)")
    ax.set_zlabel("Z axis (index offset)")

    plt.show()

