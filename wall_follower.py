import time
import numpy as np
from mbot_bridge.api import 

# Madeline Gilmore
# min distance, cross product, demo, optional small website


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum valid ray in the scan.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest valid ray in the scan.
    """
    if ranges is None or thetas is None:
        return None, None

    ranges = np.asarray(ranges, dtype=float)
    thetas = np.asarray(thetas, dtype=float)

    valid_idx = (ranges > 0).nonzero()[0]
    if len(valid_idx) == 0:
        return None, None

    ranges = ranges[valid_idx]
    thetas = thetas[valid_idx]

    min_idx = np.argmin(ranges)
    return float(ranges[min_idx]), float(thetas[min_idx])


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = []

    res.append(v1[1] * v2[2] - v1[2] * v2[1])
    res.append(v1[2] * v2[0] - v1[0] * v2[2])
    res.append(v1[0] * v2[1] - v1[1] * v2[0])
    return res


robot = MBot()
setpoint = 0  # TODO: Pick your setpoint.
kp = 0.5  
# TODO: Declare any other variables you might need here.

try:
    while True:
        # Read the latest lidar scan.
        ranges, thetas = robot.read_lidar()

        # Find the shortest valid LiDAR ray.
        if ranges is None or thetas is None or len(ranges) == 0:
            robot.stop()
            time.sleep(0.1)
            continue

        min_dist, min_angle = find_min_dist(ranges, thetas)
        if min_dist is None:
            robot.stop()
            time.sleep(0.1)
            continue

        # vector pointing toward the nearest wall
        v_to_wall = np.array([
            np.cos(min_angle),
            np.sin(min_angle),
            0.0,
        ])
        v_up = np.array([0.0, 0.0, 1.0])
        v_forward = cross_product(v_to_wall, v_up)

        # P-control
        distance_error = min_dist - setpoint
        v_correction = kp * distance_error * v_to_wall

        # wall tangent  with distance correction
        v_control = np.asarray(v_forward) + v_correction
        robot.drive(v_control[0], v_control[1], 0.0)

        print(f"Nearest ray: distance={min_dist:.2f} m, angle={min_angle:.2f} rad")

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)
except:
    # Catch any exception, including the user quitting, and stop the robot.
    robot.stop()
