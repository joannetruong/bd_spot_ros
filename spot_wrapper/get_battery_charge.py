import time

from spot_wrapper.spot import Spot


def main(spot: Spot):
    while True:
        charge = spot.get_battery_charge()
        spot.loginfo(f"Battery level: {charge}")
        time.sleep(1 / 30.0)


if __name__ == "__main__":
    spot = Spot("BatteryClient")
    main(spot)
