import argparse

import numpy as np
from spot import Spot

# End time for the command in seconds.
VEL_END_TIME = 1.0

# Velocity to move Spot (m/s)
X_VEL = 0.0
Y_VEL = 0.5
T_VEL = 0.0

NUM_TRIALS = 3


def velocity_test():
    ## Forward 1m
    spot.loginfo("MOVING FORWARD")
    spot.set_base_velocity(X_VEL, 0.0, 0.0, VEL_END_TIME)
    ## Right 1m
    spot.loginfo("MOVING RIGHT")
    spot.set_base_velocity(0.0, -Y_VEL, 0.0, VEL_END_TIME)
    ## Backward 1m
    spot.loginfo("MOVING BACKWARD")
    spot.set_base_velocity(-X_VEL, 0.0, 0.0, VEL_END_TIME)
    ## Left 1m
    spot.loginfo("MOVING LEFT")
    spot.set_base_velocity(0.0, Y_VEL, 0.0, VEL_END_TIME)

def forward_step():
    #spot.loginfo("Moving Forward")
    spot.set_base_velocity(X_VEL, 0.0, 0.0, VEL_END_TIME)

def backward_step():
    spot.loginfo("Moving Forward")
    spot.set_base_velocity(-X_VEL, 0.0, 0.0, VEL_END_TIME)


def main(spot: Spot):
    spot.power_on()
    spot.blocking_stand()



    home_pos = spot.get_robot_position()
    home_rpy = spot.get_robot_rpy()
    steps_per_pass = 10
    num_passes = 20
    total_steps = 0
    data_file = open("noise_data.txt", "w")
    print('home')
    print(home_pos)



    
    for _ in range(num_passes):
        for _ in range(steps_per_pass):


            total_steps += 1
            init_pos = spot.get_robot_position()
            init_rpy = spot.get_robot_rpy()
            spot.set_base_velocity(0.0, Y_VEL, 0.0,  VEL_END_TIME)
            cur_pos = spot.get_robot_position()
            cur_rpy = spot.get_robot_rpy()

            
            line = ('-------------- step ' + str(total_steps) + ' ----------\n')
            line += ('init pos: ' + str(init_pos) + '\n')
            line += ('init rpy: ' + str(init_rpy) + '\n')
            line += ('cur pos: '  + str(cur_pos)  + '\n')
            line += ('cur rpy: '  + str(cur_rpy)  + '\n')
            line += ('cmd: x,y,w,t' + str(np.array([X_VEL, Y_VEL, T_VEL, VEL_END_TIME])) + '\n')
            
            
            data_file.write(line)
            print('Steps: '  + str(total_steps) + '/' + str(2 * steps_per_pass * num_passes))

        spot.set_global_base_position(cur_pos[0],cur_pos[1], cur_rpy[-1], 10.0)

        for _ in range(steps_per_pass):


            total_steps += 1
            init_pos = spot.get_robot_position()
            init_rpy = spot.get_robot_rpy()
            spot.set_base_velocity(0.0, -Y_VEL, 0.0,  VEL_END_TIME)
            cur_pos = spot.get_robot_position()
            cur_rpy = spot.get_robot_rpy()

            
            line = ('-------------- step ' + str(total_steps) + ' ----------\n')
            line += ('init pos: ' + str(init_pos) + '\n')
            line += ('init rpy: ' + str(init_rpy) + '\n')
            line += ('cur pos: '  + str(cur_pos)  + '\n')
            line += ('cur rpy: '  + str(cur_rpy)  + '\n')
            line += ('cmd: x,y,w,t' + str(np.array([-X_VEL, -Y_VEL, T_VEL, VEL_END_TIME])) + '\n')
            
            
            data_file.write(line)
            print('Steps: '  + str(total_steps) + '/' + str(2 * steps_per_pass * num_passes))

     
        spot.set_global_base_position(home_pos[0],home_pos[1], home_rpy[-1], 10.0)
        print(spot.get_robot_position())




    data_file.close()




if __name__ == "__main__":
    spot = Spot("DataCollection")
    with spot.get_lease(hijack=True) as lease:
        main(spot)
