import argparse

import numpy as np
from spot import Spot

# End time for the command in seconds.
VEL_END_TIME = 1.0

# Velocity to move Spot (m/s)
X_VEL = 0.5
Y_VEL = 0.5
T_VEL = 0.3

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


def main(spot: Spot):
    spot.power_on()
    spot.blocking_stand()
    for _ in range(NUM_TRIALS):
        velocity_test()


if __name__ == "__main__":
    spot = Spot("DataCollection")
    with spot.get_lease(hijack=True) as lease:
        main(spot)
