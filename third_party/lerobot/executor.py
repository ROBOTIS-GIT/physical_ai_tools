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

"""
LeRobot Executor - Decorator-based executor for LeRobot training and inference.

Services (via RobotServiceServer):
    /lerobot/train             - Start training (async)
    /lerobot/infer             - Load policy for inference
    /lerobot/get_action_chunk  - Get action chunk (on-demand)
    /lerobot/stop              - Stop training/inference
    /lerobot/status            - Get state and progress (built-in)
"""
import atexit
import logging
import os

from robot_client import RobotServiceServer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

server = RobotServiceServer(
    name="lerobot",
    router_ip=os.environ.get("ZENOH_ROUTER_IP", "127.0.0.1"),
    router_port=int(os.environ.get("ZENOH_ROUTER_PORT", "7447")),
    domain_id=int(os.environ.get("ROS_DOMAIN_ID", "30")),
    node_name="lerobot_executor",
)


@server.on_train
def train_callback(request):
    from training import run_training
    run_training(server, request)


@server.on_load_policy
def load_policy_callback(request):
    from inference import load_policy
    return load_policy(server, request)


@server.on_get_action
def get_action_callback(request):
    from inference import get_action_chunk
    return get_action_chunk(server, request)


@server.on_stop
def stop_callback():
    from inference import cleanup_inference
    from training import cleanup_training
    cleanup_inference()
    cleanup_training()


if __name__ == "__main__":
    atexit.register(server._cleanup)
    server.spin()
