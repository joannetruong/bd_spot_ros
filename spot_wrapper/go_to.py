import argparse
import sys
import time

from spot_wrapper.spot import Spot


def main(spot: Spot):
    """Make Spot stand"""
    spot.power_on()
    spot.blocking_stand()

    # x, y, yaw
    spot.set_base_position(
        float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), 10.0
    )
    # Wait 5 seconds to before powering down...
    while True:
        pass
    time.sleep(5)
    spot.power_off()


if __name__ == "__main__":
    spot = Spot("GoToClient")
    with spot.get_lease(hijack=True) as lease:
        main(spot)
