import argparse

import numpy as np
from spot import Spot

# Meters to move Spot (relative position)
MOVE_AMT_X = 1.0
MOVE_AMT_Y = 1.0
# End time for the command in seconds.
END_TIME = 3.0

# Velocity to move Spot (m/s)
X_VEL = 0.5
Y_VEL = 0.5

NUM_TRIALS = 3


def position_test():
    ## Forward 1m
    spot.loginfo("MOVING FORWARD")
    spot.set_base_position(MOVE_AMT_X, 0.0, END_TIME)
    ## Right 1m
    spot.loginfo("MOVING RIGHT")
    spot.set_base_position(0.0, -MOVE_AMT_Y, END_TIME)
    ## Backward 1m
    spot.loginfo("MOVING BACKWARD")
    spot.set_base_position(-MOVE_AMT_X, 0.0, END_TIME)
    ## Left 1m
    spot.loginfo("MOVING LEFT")
    spot.set_base_position(0.0, MOVE_AMT_Y, END_TIME)


def velocity_test():
    vel_x_end_time = MOVE_AMT_X / X_VEL
    vel_y_end_time = MOVE_AMT_Y / Y_VEL

    ## Forward 1m
    spot.loginfo("MOVING FORWARD")
    spot.set_base_velocity(X_VEL, 0.0, 0.0, vel_x_end_time)
    ## Right 1m
    spot.loginfo("MOVING RIGHT")
    spot.set_base_velocity(0.0, -Y_VEL, 0.0, vel_y_end_time)
    ## Backward 1m
    spot.loginfo("MOVING BACKWARD")
    spot.set_base_velocity(-X_VEL, 0.0, 0.0, vel_x_end_time)
    ## Left 1m
    spot.loginfo("MOVING LEFT")
    spot.set_base_velocity(0.0, Y_VEL, 0.0, vel_y_end_time)


def main(spot: Spot):
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--velocity", action="store_true")
    args = parser.parse_args()

    spot.power_on()
    spot.blocking_stand()
    for _ in range(NUM_TRIALS):
        if args.velocity:
            velocity_test()
        else:
            position_test()


if __name__ == "__main__":
    spot = Spot("LocalizationTest")
    with spot.get_lease(hijack=True) as lease:
        main(spot)
