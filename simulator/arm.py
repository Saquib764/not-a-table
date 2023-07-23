import numpy as np
from copy import deepcopy as copy

keypoints = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
targets = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
target_speeds = [0, 0, 0, 0]
target_directions = [[1,1], [1,1], [1,1], [1,1]]
angles_at_keypoints = [0, 0, 0, 0]
max_speeds = [[0, 0], [0, 0], [0, 0], [0, 0]]
should_stop = [False, False, False, False]
curent_target_index = 2

current_position = [0, 0]
current_acceleration = [0, 0]
current_speed = [0, 0]
error = 0

MAX_SPEED = 50
MAX_ACCELERATION = 3 * MAX_SPEED
N = 3 * 200 * 32 / (2 * np.pi)

def to_xy(a1, a2):
    ARM = 0.33
    x = ARM * np.cos(a1) + ARM * np.cos(a1 + a2)
    y = ARM * np.sin(a1) + ARM * np.sin(a1 + a2)
    return [x, y]

def add_point_to_trajectory(pt):
    global keypoints, curent_target_index, keypoints, targets, max_speeds
    _pt = to_xy(pt[0], pt[1])
    _lpt = to_xy(keypoints[-1][0], keypoints[-1][1])
    angle_to_new_point = np.arctan2(_pt[1] - _lpt[1], _pt[0] - _lpt[0]) * 180 / np.pi
    target_speed_to_new_point = -1
    if abs(angle_to_new_point - angles_at_keypoints[-1]) > 20:
        # Stop
        target_speed_to_new_point = 0
    
    # shift all points to the left
    for i in range(len(keypoints) - 1):
        keypoints[i] = copy(keypoints[i + 1])
        targets[i] = copy(targets[i + 1])
    
    for i in range(len(angles_at_keypoints) - 1):
        angles_at_keypoints[i] = copy(angles_at_keypoints[i + 1])
        target_speeds[i] = copy(target_speeds[i + 1])
        target_directions[i] = copy(target_directions[i + 1])
        max_speeds[i] = copy(max_speeds[i + 1])
        should_stop[i] = should_stop[i + 1]
    # add new point to the end
    keypoints[-1] = pt
    targets[-1] = [int(pt[0] * N), int(pt[1] * N)]
    target_speeds[-1] = target_speed_to_new_point
    if keypoints[-1][0] - keypoints[-2][0] > 0:
        target_directions[-1][0] = 1
    elif keypoints[-1][0] - keypoints[-2][0] < 0:
        target_directions[-1][0] = -1
    
    if keypoints[-1][1] - keypoints[-2][1] > 0:
        target_directions[-1][1] = 1
    elif keypoints[-1][1] - keypoints[-2][1] < 0:
        target_directions[-1][1] = -1

    curent_target_index -= 1
    angles_at_keypoints[-1] = angle_to_new_point

    if target_directions[-1][0] != target_directions[-2][0] or target_directions[-1][1] != target_directions[-2][1]:
        should_stop[-1] = True
    else:
        should_stop[-1] = target_speed_to_new_point != 0


    distance_to_target = [abs(targets[-1][0] - targets[-2][0]), abs(targets[-1][1] - targets[-2][1])]
    max_speeds[-1][0] = MAX_SPEED * (1.0 * distance_to_target[0]) / (distance_to_target[0] + 0.001)
    max_speeds[-1][1] = MAX_SPEED * (1.0 * distance_to_target[1]) / (distance_to_target[0] + 0.001)
    if max_speeds[-1][1] > MAX_SPEED:
        max_speeds[-1][0] = MAX_SPEED * (1.0 * distance_to_target[0]) / (distance_to_target[1] + 0.001)
        max_speeds[-1][1] = MAX_SPEED * (1.0 * distance_to_target[1]) / (distance_to_target[1] + 0.001)
    
    max_speeds[-1][0] = np.ceil(max_speeds[-1][0])
    max_speeds[-1][1] = np.ceil(max_speeds[-1][1])

    print("keypoints: ", keypoints)
    print("targets: ", targets)
    print("angles_at_keypoints: ", angles_at_keypoints)
    print("targets speed: ", target_speeds)
    print("directions: ", target_directions)
    print("max speed: ", max_speeds)
    print("should_stop: ", should_stop)
    print("curent_target_index: ", curent_target_index)
    print(" ")
    pass

def follow_trajectory():
    global curent_target_index, current_acceleration, max_speeds, error
    if curent_target_index >= len(targets):
        return False, True
    displacement_to_target = [targets[curent_target_index][0] - current_position[0], targets[curent_target_index][1] - current_position[1]]


    if displacement_to_target[0] * target_directions[curent_target_index -1 ][0] < MAX_SPEED * 0.5 and displacement_to_target[1]*target_directions[curent_target_index -1 ][1] < MAX_SPEED * 0.5:
        curent_target_index += 1
        print("curent_target_index: ", curent_target_index)
        return True, False
    distance_to_target = [abs(displacement_to_target[0]) + 0.001, abs(displacement_to_target[1]) + 0.001]

    
    current_acceleration[0] = MAX_ACCELERATION
    current_acceleration[1] = MAX_ACCELERATION * distance_to_target[1] / distance_to_target[0]

    prev_directions = [1, 1]
    prev_directions[0] = 1 if current_speed[0]>=0 else -1
    prev_directions[1] = 1 if current_speed[1]>=0 else -1

    original_displacement = [
        targets[curent_target_index][0] - targets[curent_target_index - 1][0],
        targets[curent_target_index][1] - targets[curent_target_index - 1][1]]
    displacement_to_target = [
        targets[curent_target_index][0] - current_position[0],
        targets[curent_target_index][1] - current_position[1]]

    error_correction_index = 0
    error = 0
    if abs(original_displacement[0]) > 10 and \
        abs(original_displacement[1]) > 10 and \
        abs(displacement_to_target[0]) > 10 and \
        abs(displacement_to_target[1]) > 10:
        # correct the motor with smaller distance
        r1 = abs(1- displacement_to_target[0] / original_displacement[0])
        r2 = abs(1 - displacement_to_target[1] / original_displacement[1])
        if r1 < r2:
            # 1 id faster, slow id down
            error_correction_index = 1
        # error_correction_index = 0 if abs(original_displacement[0]) < abs(original_displacement[1]) else 1
        reference_index = 1 if error_correction_index == 0 else 0
        expected_position_of_error_index = original_displacement[error_correction_index] * abs(displacement_to_target[reference_index]/ original_displacement[reference_index])

        error =  (expected_position_of_error_index - displacement_to_target[error_correction_index]) * target_directions[curent_target_index -1][error_correction_index]
        # print(error)

        # error is always positive by design

    # print("error: ", error, error_correction_index, abs(displacement_to_target[0] / original_displacement[0] + 0.01), abs(displacement_to_target[1] / original_displacement[1]))

    _max_speed = max_speeds[curent_target_index-1]
    
    current_acceleration[0] = (_max_speed[0] * target_directions[curent_target_index-1][0] - current_speed[0]) * 2
    current_acceleration[1] = (_max_speed[1] * target_directions[curent_target_index-1][1] - current_speed[1]) * 2


    if error >= 1:
        current_acceleration[error_correction_index] = -0.2 * current_speed[error_correction_index]
        pass
    
    if (displacement_to_target[0] * target_directions[curent_target_index -1 ][0] < 0.3 * abs(current_speed[0]) or displacement_to_target[1] * target_directions[curent_target_index -1 ][1] < 0.3 * abs(current_speed[1])) and should_stop[curent_target_index - 1 ]:
        if abs(current_speed[0]) > 0.3 * _max_speed[0]:
            current_acceleration[0] = -0.9 * current_speed[0]
        if abs(current_speed[1]) > 0.3 * _max_speed[1]:
            current_acceleration[1] = -0.9 * current_speed[1]
        pass

    # last few steps
    # if displacement_to_target[0] * target_directions[curent_target_index -1 ][0] > 0 and abs(current_speed[0]) < 0.02:
    #     current_acceleration[0] = 0.9 * displacement_to_target[0]
    
    # if displacement_to_target[1] * target_directions[curent_target_index -1 ][1] > 0 and abs(current_speed[1]) < 0.02:
    #     current_acceleration[1] = 0.9 * displacement_to_target[1]

    # print("current_speed: ", current_speed)
    # print("max speed: ", _max_speed)
    # print("current_acceleration: ", current_acceleration)

    # print("current_position: ", displacement_to_target[0] * target_directions[curent_target_index -1 ][0], displacement_to_target[1]*target_directions[curent_target_index -1 ][1])
    return False, False

p = []
v = []
mv = []
a = []
e = []

dt = 0.01
total_error = 0

def move():
    global current_position, current_speed, max_speeds, curent_target_index, total_error
    if curent_target_index >= len(targets):
        return [current_position[0]/N, current_position[1]/N]
    directions = [1, 1]

    _max_speeds = [max_speeds[curent_target_index-1][0], max_speeds[curent_target_index -1][1]]
    current_speed[0] += current_acceleration[0] * dt
    current_speed[1] += current_acceleration[1] * dt

    directions[0] = 1 if current_speed[0]>=0 else -1
    directions[1] = 1 if current_speed[1]>=0 else -1

    current_speed[0] = min(abs(current_speed[0]), _max_speeds[0]) * directions[0]
    current_speed[1] = min(abs(current_speed[1]), _max_speeds[1]) * directions[1]

    current_position[0] += current_speed[0] * dt
    current_position[1] += current_speed[1] * dt

    # print("current speed", current_speed)
    # print("error", error)
    p.append(copy(current_position))
    v.append(copy(current_speed))
    mv.append(copy(_max_speeds))
    a.append(copy(current_acceleration))
    e.append(copy(error))
    total_error += error
    return [current_position[0]/N, current_position[1]/N]

