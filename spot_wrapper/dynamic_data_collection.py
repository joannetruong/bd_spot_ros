import argparse
import time

import numpy as np
from spot import Spot

# End time for the command in seconds.
VEL_END_TIME = 1.0

# Velocity to move Spot (m/s)
X_VEL = 0.5
Y_VEL = 0.5
T_VEL = 0.52

NUM_PASSES = 100
STEPS_PER_PASS = 10
SAVE_PTH = "coupled_rand_noise_data_v2/data_7.txt"


def step_robot(total_steps, data_file, negative=False):
    total_steps += 1
    init_pos = spot.get_robot_position()
    init_rpy = spot.get_robot_rpy()
    init_quat_xyzw = spot.get_robot_quat()
    init_lin_vel = spot.get_robot_linear_vel()
    init_ang_vel = spot.get_robot_angular_vel()
    cmd_x = np.random.uniform(-X_VEL, X_VEL)
    cmd_y = np.random.uniform(-Y_VEL, Y_VEL)
    cmd_t = np.random.uniform(-T_VEL, T_VEL)
    # cmd_x = np.random.uniform(0, X_VEL)
    # cmd_y = np.random.uniform(0, Y_VEL)
    # cmd_t = np.random.uniform(0, T_VEL)
    # cmd_x = 0
    # cmd_y = 0
    # cmd_t = 0
    if negative:
        cmd_x *= -1
        cmd_y *= -1
        cmd_t *= -1
    spot.set_base_velocity(cmd_x, cmd_y, cmd_t, VEL_END_TIME)
    cur_pos = spot.get_robot_position()
    cur_rpy = spot.get_robot_rpy()
    cur_quat_xyzw = spot.get_robot_quat()
    cur_lin_vel = spot.get_robot_linear_vel()
    cur_ang_vel = spot.get_robot_angular_vel()

    line = "-------------- step " + str(total_steps) + " ----------\n"
    line += "init pos: " + str(init_pos) + "\n"
    line += "init rpy: " + str(init_rpy) + "\n"
    line += "init quat xyzw: " + str(init_quat_xyzw) + "\n"
    line += "init lin vel: " + str(init_lin_vel) + "\n"
    line += "init ang vel: " + str(init_ang_vel) + "\n"
    line += "cur pos: " + str(cur_pos) + "\n"
    line += "cur rpy: " + str(cur_rpy) + "\n"
    line += "cur quat xyzw: " + str(cur_quat_xyzw) + "\n"
    line += "cur lin vel: " + str(cur_lin_vel) + "\n"
    line += "cur ang vel: " + str(cur_ang_vel) + "\n"
    line += "cmd: x,y,w,t" + str(np.array([cmd_x, cmd_y, cmd_t, VEL_END_TIME])) + "\n"

    data_file.write(line)
    print("Steps: " + str(total_steps) + "/" + str(STEPS_PER_PASS * NUM_PASSES))
    return total_steps


def main(spot: Spot):
    spot.power_on()
    spot.blocking_stand()

    home_pos = spot.get_robot_position()
    home_rpy = spot.get_robot_rpy()
    total_steps = 0
    data_file = open(SAVE_PTH, "w")
    print("home")
    print(home_pos)

    for _ in range(NUM_PASSES):
        for _ in range(STEPS_PER_PASS):
            total_steps = step_robot(total_steps, data_file, negative=True)
        time.sleep(2)
        # for _ in range(STEPS_PER_PASS):
        # step_robot(total_steps, data_file, negative=True)
        spot.set_global_base_position(home_pos[0], home_pos[1], home_rpy[-1], 5.0)
        print(spot.get_robot_position())

    data_file.close()


if __name__ == "__main__":
    spot = Spot("DataCollection")
    with spot.get_lease(hijack=True) as lease:
        main(spot)
