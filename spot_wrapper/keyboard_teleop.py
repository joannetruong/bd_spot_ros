import curses
import signal
import time

import numpy as np
from spot_wrapper.spot import Spot, SpotCamIds

MOVE_INCREMENT = 0.02
TILT_INCREMENT = 5.0
BASE_ANGULAR_VEL = np.deg2rad(50)
BASE_LIN_VEL = 0.5
DOCK_ID = 520

# Where the gripper goes to upon initialization
INITIAL_POINT = np.array([0.75, 0.0, 0.35])
INITIAL_RPY = np.deg2rad([0.0, 60.0, 0.0])
KEY2BASEMOVEMENT = {
    "q": [0.0, 0.0, BASE_ANGULAR_VEL],  # turn left
    "e": [0.0, 0.0, -BASE_ANGULAR_VEL],  # turn right
    "w": [BASE_LIN_VEL, 0.0, 0.0],  # go forward
    "s": [-BASE_LIN_VEL, 0.0, 0.0],  # go backward
    "a": [0.0, BASE_LIN_VEL, 0.0],  # strafe left
    "d": [0.0, -BASE_LIN_VEL, 0.0],  # strafe right
}
INSTRUCTIONS = "Use 'wasdqe' for translating base. \n" "Press 'z' to quit.\n"


def move_to_initial(spot):
    point = INITIAL_POINT
    rpy = INITIAL_RPY

    return point, rpy


def raise_error(sig, frame):
    raise RuntimeError


def main(spot: Spot):
    """Uses IK to move the arm by setting hand poses"""
    spot.power_on()
    spot.blocking_stand()

    # Move arm to initial configuration
    point, rpy = move_to_initial(spot)
    control_arm = True

    # Start in-terminal GUI
    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    signal.signal(signal.SIGINT, raise_error)
    stdscr.addstr(INSTRUCTIONS)
    try:
        while True:
            point_rpy = np.concatenate([point, rpy])
            pressed_key = stdscr.getch()
            if pressed_key != -1:
                pressed_key = chr(pressed_key)

            if pressed_key == "z":
                # Quit
                break
            elif pressed_key == "t":
                # Toggle between controlling arm or base
                control_arm = not control_arm
                spot.loginfo(f"control_arm: {control_arm}")
                time.sleep(0.2)  # Wait before we starting listening again
            elif pressed_key == "g":
                # Grab whatever object is at the center of hand RGB camera image
                image_responses = spot.get_image_responses([SpotCamIds.HAND_COLOR])
                hand_image_response = image_responses[0]  # only expecting one image
                spot.grasp_point_in_image(hand_image_response)
                # Retract arm back to initial configuration
                point, rpy = move_to_initial(spot)
            elif pressed_key == "n":
                spot.dock(DOCK_ID)
            else:
                if pressed_key in KEY2BASEMOVEMENT:
                    # Move base
                    x_vel, y_vel, ang_vel = KEY2BASEMOVEMENT[pressed_key]
                    spot.set_base_velocity(
                        x_vel=x_vel, y_vel=y_vel, ang_vel=ang_vel, vel_time=0.5
                    )
    finally:
        spot.power_off()
        curses.echo()
        stdscr.nodelay(False)
        curses.endwin()


if __name__ == "__main__":
    spot = Spot("ArmKeyboardTeleop")
    with spot.get_lease(hijack=True) as lease:
        main(spot)
