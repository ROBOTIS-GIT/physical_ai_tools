#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

import os
import time
from dataclasses import asdict
from pprint import pformat

import rerun as rr
from typing import Optional

# from safetensors.torch import load_file, save_file
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.policies.factory import make_policy
from lerobot.common.robot_devices.control_configs import (
    CalibrateControlConfig,
    ControlConfig,
    ControlPipelineConfig,
    RecordControlConfig,
    RemoteRobotConfig,
    ReplayControlConfig,
    TeleoperateControlConfig,
)
from lerobot.common.robot_devices.control_utils import (
    control_loop,
    init_keyboard_listener,
    is_headless,
    log_control_info,
    record_episode,
    reset_environment,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
    stop_recording,
    warmup_record,
)
from lerobot.common.robot_devices.robots.utils import Robot, make_robot_from_config
from lerobot.common.robot_devices.utils import busy_wait, safe_disconnect
from lerobot.common.utils.utils import has_method, init_logging, log_say
from lerobot.configs import parser

########################################################################################
# Control modes
########################################################################################


@safe_disconnect
def calibrate(robot: Robot, cfg: CalibrateControlConfig):
    # TODO(aliberts): move this code in robots' classes
    if robot.robot_type.startswith("stretch"):
        if not robot.is_connected:
            robot.connect()
        if not robot.is_homed():
            robot.home()
        return

    arms = robot.available_arms if cfg.arms is None else cfg.arms
    unknown_arms = [arm_id for arm_id in arms if arm_id not in robot.available_arms]
    available_arms_str = " ".join(robot.available_arms)
    unknown_arms_str = " ".join(unknown_arms)

    if arms is None or len(arms) == 0:
        raise ValueError(
            "No arm provided. Use `--arms` as argument with one or more available arms.\n"
            f"For instance, to recalibrate all arms add: `--arms {available_arms_str}`"
        )

    if len(unknown_arms) > 0:
        raise ValueError(
            f"Unknown arms provided ('{unknown_arms_str}'). Available arms are `{available_arms_str}`."
        )

    for arm_id in arms:
        arm_calib_path = robot.calibration_dir / f"{arm_id}.json"
        if arm_calib_path.exists():
            print(f"Removing '{arm_calib_path}'")
            arm_calib_path.unlink()
        else:
            print(f"Calibration file not found '{arm_calib_path}'")

    if robot.is_connected:
        robot.disconnect()

    if robot.robot_type.startswith("lekiwi") and "main_follower" in arms:
        print("Calibrating only the lekiwi follower arm 'main_follower'...")
        robot.calibrate_follower()
        return

    if robot.robot_type.startswith("lekiwi") and "main_leader" in arms:
        print("Calibrating only the lekiwi leader arm 'main_leader'...")
        robot.calibrate_leader()
        return

    # Calling `connect` automatically runs calibration
    # when the calibration file is missing
    robot.connect()
    robot.disconnect()
    print("Calibration is done! You can now teleoperate and record datasets!")


@safe_disconnect
def teleoperate(robot: Robot, cfg: TeleoperateControlConfig):
    control_loop(
        robot,
        control_time_s=cfg.teleop_time_s,
        fps=cfg.fps,
        teleoperate=True,
        display_data=cfg.display_data,
    )


@safe_disconnect
def record(
    robot: Robot,
    cfg: RecordControlConfig,
    teleop_step_node: Optional[object] = None,
    observation_node: Optional[object] = None,
    action_trajectory_node: Optional[object] = None,
) -> LeRobotDataset:
    # TODO(rcadene): Add option to record logs
    if cfg.resume:
        dataset = LeRobotDataset(
            cfg.repo_id,
            root=cfg.root,
        )
        if len(robot.cameras) > 0:
            dataset.start_image_writer(
                num_processes=cfg.num_image_writer_processes,
                num_threads=cfg.num_image_writer_threads_per_camera * len(robot.cameras),
            )
        sanity_check_dataset_robot_compatibility(dataset, robot, cfg.fps, cfg.video)
    else:
        # Create empty dataset or load existing saved episodes
        sanity_check_dataset_name(cfg.repo_id, cfg.policy)
        dataset = LeRobotDataset.create(
            cfg.repo_id,
            cfg.fps,
            root=cfg.root,
            robot=robot,
            use_videos=cfg.video,
            image_writer_processes=cfg.num_image_writer_processes,
            image_writer_threads=cfg.num_image_writer_threads_per_camera * len(robot.cameras),
        )

    # Load pretrained policy
    policy = None if cfg.policy is None else make_policy(cfg.policy, ds_meta=dataset.meta)

    if cfg.use_ros:
        pass
    else:
        if not robot.is_connected:
            robot.connect()

    listener, events = init_keyboard_listener()
    
    # should_create_ros = False
    if cfg.use_ros:
        # should_create_ros = teleop_step_node is None
        # if should_create_ros:
        if cfg.policy is None:
            import rclpy
            if robot.robot_type == 'omx':
                from data_collector.topic_to_data_omx import DataCollector
            elif robot.robot_type == 'ffw':
                from data_collector.topic_to_data import DataCollector
            rclpy.init()
            teleop_step_node = DataCollector()
        else:
            import rclpy
            if robot.robot_type == 'omx':
                from policy_to_trajectory.action_to_trajectory_omx import PolicyTrajectory
                from policy_to_trajectory.topic_to_observation_omx import ObservationCollector
            elif robot.robot_type == 'ffw':
                from policy_to_trajectory.action_to_trajectory import PolicyTrajectory
                from policy_to_trajectory.topic_to_observation import ObservationCollector
            rclpy.init()
            observation_node = ObservationCollector()
            action_trajectory_node = PolicyTrajectory()
 
    # Execute a few seconds without recording to:
    # 1. teleoperate the robot to move it in starting position if no policy provided,
    # 2. give times to the robot devices to connect and start synchronizing,
    # 3. place the cameras windows on screen
    enable_teleoperation = policy is None
    log_say("Warmup record", cfg.play_sounds)

    if cfg.use_ros:
        pass
    else:
        warmup_record(robot, events, enable_teleoperation, cfg.warmup_time_s, cfg.display_data, cfg.fps)

    if has_method(robot, "teleop_safety_stop"):
        robot.teleop_safety_stop()

    recorded_episodes = 0

    while True:
        if recorded_episodes >= cfg.num_episodes:
            break
        log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)

        if cfg.use_ros:
            record_episode(
                robot=robot,
                dataset=dataset,
                events=events,
                episode_time_s=cfg.episode_time_s,
                display_data=cfg.display_data,
                policy=policy,
                fps=cfg.fps,
                single_task=cfg.single_task,
                teleop_step_node=teleop_step_node,
                observation_node=observation_node,
                action_trajectory_node=action_trajectory_node,
            )
        else:
            record_episode(
                robot=robot,
                dataset=dataset,
                events=events,
                episode_time_s=cfg.episode_time_s,
                display_data=cfg.display_data,
                policy=policy,
                fps=cfg.fps,
                single_task=cfg.single_task,
            )


        # Execute a few seconds without recording to give time to manually reset the environment
        # Current code logic doesn't allow to teleoperate during this time.
        # TODO(rcadene): add an option to enable teleoperation during reset
        # Skip reset for the last episode to be recorded
        if not events["stop_recording"] and (
            (recorded_episodes < cfg.num_episodes - 1) or events["rerecord_episode"]
        ):
            log_say("Reset the environment", cfg.play_sounds)
            reset_environment(robot, events, cfg.reset_time_s, cfg.fps)

        if events["rerecord_episode"]:
            log_say("Re-record episode", cfg.play_sounds)
            events["rerecord_episode"] = False
            events["exit_early"] = False
            dataset.clear_episode_buffer()
            continue

        dataset.save_episode()
        recorded_episodes += 1

        if events["stop_recording"]:
            break

    log_say("Stop recording", cfg.play_sounds, blocking=True)

    if cfg.use_ros:
        pass
    else:
        stop_recording(robot, listener, cfg.display_data)

    if cfg.push_to_hub:
        dataset.push_to_hub(tags=cfg.tags, private=cfg.private)

    log_say("Exiting", cfg.play_sounds)
    return dataset


@safe_disconnect
def replay(
    robot: Robot,
    cfg: ReplayControlConfig,
):
    # TODO(rcadene, aliberts): refactor with control_loop, once `dataset` is an instance of LeRobotDataset
    # TODO(rcadene): Add option to record logs

    dataset = LeRobotDataset(cfg.repo_id, root=cfg.root, episodes=[cfg.episode])
    actions = dataset.hf_dataset.select_columns("action")

    if not robot.is_connected:
        robot.connect()

    log_say("Replaying episode", cfg.play_sounds, blocking=True)
    for idx in range(dataset.num_frames):
        start_episode_t = time.perf_counter()

        action = actions[idx]["action"]
        robot.send_action(action)

        dt_s = time.perf_counter() - start_episode_t
        busy_wait(1 / cfg.fps - dt_s)

        dt_s = time.perf_counter() - start_episode_t
        log_control_info(robot, dt_s, fps=cfg.fps)


def _init_rerun(control_config: ControlConfig, session_name: str = "lerobot_control_loop") -> None:
    """Initializes the Rerun SDK for visualizing the control loop.

    Args:
        control_config: Configuration determining data display and robot type.
        session_name: Rerun session name. Defaults to "lerobot_control_loop".

    Raises:
        ValueError: If viewer IP is missing for non-remote configurations with display enabled.
    """
    if (control_config.display_data and not is_headless()) or (
        control_config.display_data and isinstance(control_config, RemoteRobotConfig)
    ):
        # Configure Rerun flush batch size default to 8KB if not set
        batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
        os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size

        # Initialize Rerun based on configuration
        rr.init(session_name)
        if isinstance(control_config, RemoteRobotConfig):
            viewer_ip = control_config.viewer_ip
            viewer_port = control_config.viewer_port
            if not viewer_ip or not viewer_port:
                raise ValueError(
                    "Viewer IP & Port are required for remote config. Set via config file/CLI or disable control_config.display_data."
                )
            rr.connect_tcp(f"{viewer_ip}:{viewer_port}")
        else:
            # Get memory limit for rerun viewer parameters
            memory_limit = os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%")
            rr.spawn(memory_limit=memory_limit)

# Image Topic sub Callback 생성

# Joint State Topic sub Callback 생성

@parser.wrap()
def control_robot(cfg: ControlPipelineConfig):
    init_logging()

    robot = make_robot_from_config(cfg.robot)

    # Image Topic sub 생성
    # Joint State Topic sub 생성

    # 학습된 모델을 읽어서 Policy 생성
    # 학습된 모델을 기반으로 로봇 Observation initialization
    # 학습된 모델을 기반으로 로봇 Ouput initialization

    if robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
        robot.disconnect()


if __name__ == "__main__":
    control_robot()
