#!/usr/bin/env python3
"""Bridge script for calling /groot/infer via zenoh_ros2_sdk.

Runs inside the groot container. Called via `docker exec` from
physical_ai_server to bypass rmw_zenoh_cpp stale queryable issues.

Usage:
    python3 /app/bridge_infer.py <model_path> <embodiment_tag> <robot_type> [task_instruction]

Output (JSON):
    {"success": true, "message": "...", "action_keys": ["arm_left", ...]}
"""
import json
import sys
import os

sys.path.insert(0, os.environ.get("ZENOH_SDK_PATH", "/zenoh_sdk"))
sys.path.insert(0, "/")


def main():
    if len(sys.argv) < 4:
        print(json.dumps({"success": False, "message": "Usage: bridge_infer.py <model_path> <embodiment_tag> <robot_type> [task_instruction]", "action_keys": []}))
        sys.exit(1)

    model_path = sys.argv[1]
    embodiment_tag = sys.argv[2]
    robot_type = sys.argv[3]
    task_instruction = sys.argv[4] if len(sys.argv) > 4 else ""

    from robot_client.messages import (
        START_INFERENCE_REQUEST_DEF,
        START_INFERENCE_RESPONSE_DEF,
    )
    from zenoh_ros2_sdk import ROS2ServiceClient

    domain_id = int(os.environ.get("ROS_DOMAIN_ID", "30"))
    router_ip = os.environ.get("ZENOH_ROUTER_IP", "127.0.0.1")
    router_port = int(os.environ.get("ZENOH_ROUTER_PORT", "7447") or "7447")

    client = ROS2ServiceClient(
        service_name="/groot/infer",
        srv_type="physical_ai_interfaces/srv/StartInference",
        request_definition=START_INFERENCE_REQUEST_DEF,
        response_definition=START_INFERENCE_RESPONSE_DEF,
        domain_id=domain_id,
        router_ip=router_ip,
        router_port=router_port,
        timeout=120.0,
    )

    resp = client.call(
        model_path=model_path,
        embodiment_tag=embodiment_tag,
        robot_type=robot_type,
        task_instruction=task_instruction,
    )

    result = {
        "success": bool(resp.success),
        "message": str(resp.message),
        "action_keys": list(resp.action_keys) if hasattr(resp, "action_keys") else [],
    }
    print(json.dumps(result))


if __name__ == "__main__":
    main()
