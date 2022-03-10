import numpy as np
from spot import Spot


def main(spot: Spot):
    spot.power_on()
    spot.blocking_stand()
    # Meters to move Spot (relative position)
    move_amt_x = 1.0
    move_amt_y = 1.0
    # End time for the command in seconds.
    end_time = 1.0

    ## Forward 1m
    spot.loginfo("MOVING FORWARD")
    spot.set_base_position(move_amt_x, 0.0, end_time)
    ## Right 1m
    spot.loginfo("MOVING RIGHT")
    spot.set_base_position(0.0, -move_amt_y, end_time)
    ## Backward 1m
    spot.loginfo("MOVING BACKWARD")
    spot.set_base_position(-move_amt_x, 0.0, end_time)
    ## Left 1m
    spot.loginfo("MOVING LEFT")
    spot.set_base_position(0.0, move_amt_y, end_time)


if __name__ == "__main__":
    spot = Spot("LocalizationTest")
    with spot.get_lease() as lease:
        main(spot)
