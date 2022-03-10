# Copyright (c) 2021 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

""" Easy-to-use wrapper for properly controlling Spot """
import os
import time
from collections import OrderedDict
from enum import Enum

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import cv2
import numpy as np
from bosdyn.api import image_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateClient


class SpotCamIds:
    r"""Enumeration of types of cameras."""

    BACK_DEPTH = "back_depth"
    BACK_DEPTH_IN_VISUAL_FRAME = "back_depth_in_visual_frame"
    BACK_FISHEYE = "back_fisheye_image"
    FRONTLEFT_DEPTH = "frontleft_depth"
    FRONTLEFT_DEPTH_IN_VISUAL_FRAME = "frontleft_depth_in_visual_frame"
    FRONTLEFT_FISHEYE = "frontleft_fisheye_image"
    FRONTRIGHT_DEPTH = "frontright_depth"
    FRONTRIGHT_DEPTH_IN_VISUAL_FRAME = "frontright_depth_in_visual_frame"
    FRONTRIGHT_FISHEYE = "frontright_fisheye_image"
    HAND_COLOR = "hand_color_image"
    HAND_COLOR_IN_HAND_DEPTH_FRAME = "hand_color_in_hand_depth_frame"
    HAND_DEPTH = "hand_depth"
    HAND_DEPTH_IN_HAND_COLOR_FRAME = "hand_depth_in_hand_color_frame"
    HAND = "hand_image"
    LEFT_DEPTH = "left_depth"
    LEFT_DEPTH_IN_VISUAL_FRAME = "left_depth_in_visual_frame"
    LEFT_FISHEYE = "left_fisheye_image"
    RIGHT_DEPTH = "right_depth"
    RIGHT_DEPTH_IN_VISUAL_FRAME = "right_depth_in_visual_frame"
    RIGHT_FISHEYE = "right_fisheye_image"


SHOULD_ROTATE = [
    SpotCamIds.FRONTLEFT_DEPTH,
    SpotCamIds.FRONTRIGHT_DEPTH,
]

JOINT_NAMES = [
    "fl.hx",
    "fl.hy",
    "fl.kn",
    "fr.hx",
    "fr.hy",
    "fr.kn",
    "hl.hx",
    "hl.hy",
    "hl.kn",
    "hr.hx",
    "hr.hy",
    "hr.kn",
]


class Spot:
    def __init__(self, client_name_prefix):
        bosdyn.client.util.setup_logging()
        sdk = bosdyn.client.create_standard_sdk(client_name_prefix)
        robot = sdk.create_robot(os.environ["HOSTNAME"])
        robot.authenticate(os.environ["SPOT_USERNAME"], os.environ["SPOT_ADMIN_PW"])
        robot.time_sync.wait_for_sync()
        self.robot = robot
        self.command_client = None
        self.spot_lease = None

        # Get clients
        self.command_client = robot.ensure_client(
            RobotCommandClient.default_service_name
        )
        self.image_client = robot.ensure_client(ImageClient.default_service_name)
        self.manipulation_api_client = robot.ensure_client(
            ManipulationApiClient.default_service_name
        )
        self.robot_state_client = robot.ensure_client(
            RobotStateClient.default_service_name
        )

    def get_lease(self, hijack=False):
        # Make sure a lease for this client isn't already active
        assert self.spot_lease is None
        self.spot_lease = SpotLease(self, hijack=hijack)
        return self.spot_lease

    def is_estopped(self):
        return self.robot.is_estopped()

    def power_on(
        self, timeout_sec=20, service_name=RobotCommandClient.default_service_name
    ):
        self.robot.power_on(timeout_sec=timeout_sec)
        assert self.robot.is_powered_on(), "Robot power on failed."
        self.loginfo("Robot powered on.")

    def power_off(self, cut_immediately=False, timeout_sec=20):
        self.loginfo("Powering robot off...")
        self.robot.power_off(cut_immediately=cut_immediately, timeout_sec=timeout_sec)
        assert not self.robot.is_powered_on(), "Robot power off failed."
        self.loginfo("Robot safely powered off.")

    def blocking_stand(self, timeout_sec=10):
        self.loginfo("Commanding robot to stand (blocking)...")
        blocking_stand(self.command_client, timeout_sec=timeout_sec)
        self.loginfo("Robot standing.")

    def loginfo(self, *args, **kwargs):
        self.robot.logger.info(*args, **kwargs)

    def get_image_responses(self, sources, quality=None):
        """Retrieve images from Spot's cameras
        :param sources: list containing camera uuids
        :param quality: either an int or a list specifying what quality each source
            should return its image with
        :return: list containing bosdyn image response objects
        """
        if quality is not None:
            if isinstance(quality, int):
                quality = [quality] * len(sources)
            else:
                assert len(quality) == len(sources)
            sources = [build_image_request(src, q) for src, q in zip(sources, quality)]
            image_responses = self.image_client.get_image(sources)
        else:
            image_responses = self.image_client.get_image_from_sources(sources)

        return image_responses

    def set_base_velocity(self, x_vel, y_vel, ang_vel, vel_time, params=None):
        body_tform_goal = math_helpers.SE2Velocity(x=x_vel, y=y_vel, angular=ang_vel)
        if params is None:
            params = spot_command_pb2.MobilityParams(
                obstacle_params=spot_command_pb2.ObstacleParams(
                    disable_vision_body_obstacle_avoidance=False,
                    disable_vision_foot_obstacle_avoidance=False,
                    disable_vision_foot_constraint_avoidance=False,
                    obstacle_avoidance_padding=0.05,  # in meters
                )
            )
        robot_cmd = RobotCommandBuilder.synchro_velocity_command(
            v_x=body_tform_goal.linear_velocity_x,
            v_y=body_tform_goal.linear_velocity_y,
            v_rot=body_tform_goal.angular_velocity,
            params=params,
        )
        end_time = time.time() + vel_time
        cmd_id = self.command_client.robot_command(
            robot_cmd, end_time_secs=time.time() + vel_time
        )
        while time.time() < end_time:
            pass
        return cmd_id

    def set_base_position(self, x_pos, y_pos, end_time, params=None):
        vision_tform_body = get_vision_tform_body(
            self.get_robot_kinematic_state().transforms_snapshot
        )
        body_tform_goal = math_helpers.SE3Pose(
            x=x_pos, y=y_pos, z=0, rot=math_helpers.Quat()
        )

        vision_tform_goal = vision_tform_body * body_tform_goal
        if params is None:
            params = spot_command_pb2.MobilityParams(
                obstacle_params=spot_command_pb2.ObstacleParams(
                    disable_vision_body_obstacle_avoidance=False,
                    disable_vision_foot_obstacle_avoidance=False,
                    disable_vision_foot_constraint_avoidance=False,
                    obstacle_avoidance_padding=0.05,  # in meters
                )
            )
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=vision_tform_goal.x,
            goal_y=vision_tform_goal.y,
            goal_heading=vision_tform_goal.rot.to_yaw(),
            frame_name=VISION_FRAME_NAME,
            params=params,
        )
        end_time = time.time() + end_time
        cmd_id = self.command_client.robot_command(robot_cmd, end_time_secs=end_time)
        while time.time() < end_time:
            pass
        return cmd_id

    def get_robot_foot_state(self):
        return self.robot_state_client.get_robot_state().foot_state

    def get_robot_kinematic_state(self):
        return self.robot_state_client.get_robot_state().kinematic_state

    def get_robot_state(self):
        robot_state_kin = self.get_robot_kinematic_state()
        robot_state = get_vision_tform_body(robot_state_kin.transforms_snapshot)
        return robot_state

    def get_robot_joint_states(self):
        robot_state_kin = self.get_robot_kinematic_state()
        joint_states = OrderedDict(
            {i.name: i for i in robot_state_kin.joint_states if i.name in JOINT_NAMES}
        )
        # joint_states = robot_state_kin.joint_states
        return joint_states

    def get_robot_position(self):
        robot_state = self.get_robot_state()
        return [robot_state.x, robot_state.y, robot_state.z]

    def get_robot_quat(self):
        robot_state = self.get_robot_state()
        robot_quat = robot_state.rotation
        return [robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w]

    def get_robot_rpy(self):
        robot_state = self.get_robot_state()
        # returns as yaw, pitch, roll. Reverse the list to get it in roll, pitch, yaw form
        return math_helpers.quat_to_eulerZYX(robot_state.rotation)[::-1]

    def get_robot_velocity(self, frame="vision"):
        robot_state_kin = self.get_robot_kinematic_state()
        if frame == "odom":
            robot_velocity = robot_state_kin.velocity_of_body_in_odom
        elif frame == "vision":
            robot_velocity = robot_state_kin.velocity_of_body_in_vision
        return robot_velocity

    def get_robot_linear_vel(self, frame="vision"):
        robot_velocity = self.get_robot_velocity(frame)
        linear_vel = np.array(
            [robot_velocity.linear.x, robot_velocity.linear.y, robot_velocity.linear.z]
        )
        return linear_vel

    def get_robot_linear_vel_local(self, frame="vision"):
        linear_vel = self.get_robot_linear_vel(frame)
        yaw = self.get_robot_rpy()[-1]
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        return np.linalg.inv(R) @ linear_vel

    def get_robot_angular_vel(self, frame="vision"):
        robot_velocity = self.get_robot_velocity(frame)
        return [
            robot_velocity.angular.x,
            robot_velocity.angular.y,
            robot_velocity.angular.z,
        ]

    def get_base_transform_to(self, child_frame):
        kin_state = self.robot_state_client.get_robot_state().kinematic_state
        kin_state = kin_state.transforms_snapshot.child_to_parent_edge_map.get(
            child_frame
        ).parent_tform_child
        return kin_state.position, kin_state.rotation


class SpotLease:
    """
    A class that supports execution with Python's "with" statement for safe return of
    the lease and settle-then-estop upon exit. Grants control of the Spot's motors.
    """

    def __init__(self, spot, hijack=False):
        self.lease_client = spot.robot.ensure_client(
            bosdyn.client.lease.LeaseClient.default_service_name
        )
        if hijack:
            self.lease = self.lease_client.take()
        else:
            self.lease = self.lease_client.acquire()
        self.lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(self.lease_client)
        self.spot = spot

    def __enter__(self):
        return self.lease

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Exit the LeaseKeepAlive object
        self.lease_keep_alive.__exit__(exc_type, exc_val, exc_tb)
        # Return the lease
        self.lease_client.return_lease(self.lease)
        self.spot.loginfo("Returned the lease.")
        # Clear lease from Spot object
        self.spot.spot_lease = None


def image_response_to_cv2(image_response, reorient=True):
    if image_response.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
    else:
        dtype = np.uint8
    # img = np.fromstring(image_response.shot.image.data, dtype=dtype)
    img = np.frombuffer(image_response.shot.image.data, dtype=dtype)
    if image_response.shot.image.format == image_pb2.Image.FORMAT_RAW:
        img = img.reshape(
            image_response.shot.image.rows, image_response.shot.image.cols
        )
    else:
        img = cv2.imdecode(img, -1)
    if reorient and image_response.source.name in SHOULD_ROTATE:
        img = np.rot90(img, k=3)
    return img


def scale_depth_img(img, min_depth=0.0, max_depth=10.0, as_img=False):
    min_depth, max_depth = min_depth * 1000, max_depth * 1000
    img_copy = np.clip(img.astype(np.float32), a_min=min_depth, a_max=max_depth)
    img_copy = (img_copy - min_depth) / (max_depth - min_depth)
    if as_img:
        img_copy = cv2.cvtColor((255.0 * img_copy).astype(np.uint8), cv2.COLOR_GRAY2BGR)

    return img_copy


def wrap_heading(heading):
    """Ensures input heading is between -180 an 180; can be float or np.ndarray"""
    return (heading + np.pi) % (2 * np.pi) - np.pi
