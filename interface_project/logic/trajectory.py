# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : trajectory.py
# Description : Path management, updating and CSV export
# ============================================================


import math
import csv
from datetime import datetime

# ===== Physical parameters ===== 
L = 0.35  # wheel gauge (m)
v_lin = 0.035  # linear speed m/s  (SPEED 15000)
omega_rot = 0.2  # angular speed rad/s  (SPEED 15000)

# ===== Stored trajectory =====
trajectory_data = [(0.0, 0.0, 0.0)]  # x, y, theta
_last_command_time = None

def reset_trajectory():
    global trajectory_data, _last_command_time
    trajectory_data = [(0.0, 0.0, 0.0)]
    _last_command_time = None

def get_trajectory():
    return trajectory_data

def time_delta():
    global _last_command_time
    now = datetime.now()
    if _last_command_time is None:
        delta_t = 0.0
    else:
        delta = now - _last_command_time
        delta_t = delta.total_seconds()
    _last_command_time = now
    return delta_t

def update_trajectory(left_speed: int, right_speed: int):
    global trajectory_data
    delta_t = time_delta()
    if delta_t == 0.0:
        return

    x, y, theta = trajectory_data[-1]

    if left_speed * right_speed > 0:  # Rotation
        dtheta = omega_rot * delta_t
        if left_speed < 0:
            theta += dtheta  # left rotation
        else:
            theta -= dtheta  # Right rotation 
    elif left_speed * right_speed == 0:
        return
    else:  # Translation
        distance = v_lin * delta_t
        if left_speed > 0:
            x += distance * math.cos(theta)
            y += distance * math.sin(theta)
        else:
            x -= distance * math.cos(theta)
            y -= distance * math.sin(theta)

    trajectory_data.append((x, y, theta))

def export_trajectory_to_csv(filepath: str) -> bool:
    try:
        with open(filepath, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(["x (m)", "y (m)", "theta (rad)"])
            for point in trajectory_data:
                writer.writerow(point)
        return True
    except Exception as e:
        print(f"Export error : {e}")
        return False