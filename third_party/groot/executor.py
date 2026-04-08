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
GR00T N1.6 Executor - Decorator-based executor for GR00T finetuning and inference.

Services (via RobotServiceServer):
    /groot/train             - Start finetuning (async)
    /groot/infer             - Load policy for inference
    /groot/get_action_chunk  - Get action chunk (on-demand)
    /groot/stop              - Stop training/inference
    /groot/status            - Get state and progress (built-in)
"""
import atexit
import logging
import os

from inference import GR00TInference
from robot_client import RobotServiceServer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

server = RobotServiceServer(
    name="groot",
    router_ip=os.environ.get("ZENOH_ROUTER_IP", "127.0.0.1"),
    router_port=int(os.environ.get("ZENOH_ROUTER_PORT", "7447") or "7447"),
    domain_id=int(os.environ.get("ROS_DOMAIN_ID", "30")),
    node_name="groot_executor",
)

inference = GR00TInference()


@server.on_train
def train_callback(request):
    from training import run_training
    run_training(server, request)


@server.on_load_policy
def load_policy_callback(request):
    return inference.load_policy(request)


@server.on_get_action
def get_action_callback(request):
    return inference.get_action_chunk(request)


@server.on_stop
def stop_callback():
    from training import cleanup_training
    inference.cleanup()
    cleanup_training()


if __name__ == "__main__":
    # Start TCP bridge server in the SAME process as the executor.
    # It calls inference handlers directly (no Zenoh), completely
    # bypassing stale queryable routing issues.
    from bridge_server import start_bridge_server

    def _stop_all():
        from training import cleanup_training
        inference.cleanup()
        cleanup_training()

    start_bridge_server(port=9100, inference=inference, stop_fn=_stop_all)

    atexit.register(server._cleanup)
    server.spin()
