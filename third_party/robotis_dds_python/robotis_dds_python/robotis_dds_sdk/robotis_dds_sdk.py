#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Robotis DDS Python SDK + DDS Inference Server
# High-level wrapper for DDS-based robot communication + Inference integration
#
# Author: Heewon Lee, Dongyun Kim
# License: Apache 2.0

import threading
import time
import numpy as np
import cv2
import torch
from io import BytesIO

from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode

from robotis_dds_python.robotis_dds_core.idl.sensor_msgs.msg import (
    CompressedImage_,
    Image_,
    JointState_,
    BatteryState_,
)
from robotis_dds_python.robotis_dds_core.idl.nav_msgs.msg import Odometry_
from robotis_dds_python.robotis_dds_core.idl.geometry_msgs.msg import (Twist_, Vector3_)
from robotis_dds_python.robotis_dds_core.idl.trajectory_msgs.msg import (
    JointTrajectory_,
    JointTrajectoryPoint_,
)
from robotis_dds_python.robotis_dds_core.idl.std_msgs.msg import Header_
from robotis_dds_python.robotis_dds_core.idl.builtin_interfaces.msg import Time_

from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv import (
    Inference_Request,
    Inference_Response,
)


class RobotisDDSSDK:
    """
    Robotis DDS Python SDK
    ----------------------
    High-level wrapper around CycloneDDS for ROS2-compatible robot data exchange.
    + Inference service integration
    """

    def __init__(self, domain_id=30):
        self.node = DDSNode(
            name="robotis_sdk_node",
            domain_id=domain_id,
            network_interface="auto",
            allow_multicast=True,
        )

        self.cache = {}
        self._subscribed = {}
        self._last_inference_result = None

        self.topic_map = {
            "/camera/image": (Image_, self._image_callback),
            "/camera/image/compressed": (CompressedImage_, self._compressed_image_callback),
            "/odom": (Odometry_, self._odom_callback),
            "/joint_states": (JointState_, self._joint_state_callback),
            "/battery_state": (BatteryState_, self._battery_callback),
        }

        self.cmd_vel_pub = self.node.dds_create_publisher("/cmd_vel", Twist_)
        self.joint_traj_pub = self.node.dds_create_publisher("/joint_trajectory", JointTrajectory_)

        self.spin_thread = threading.Thread(target=self.node.dds_spin, daemon=True)
        self.spin_thread.start()

    def _ensure_subscription(self, topic_name: str):
        if topic_name in self._subscribed:
            return
        if topic_name not in self.topic_map:
            print(f"[RobotisDDSSDK] Unknown topic: {topic_name}")
            return
        msg_type, cb = self.topic_map[topic_name]
        self.node.dds_create_subscription(topic_name, msg_type, cb)
        self._subscribed[topic_name] = True
        print(f"[RobotisDDSSDK] Subscribed to {topic_name}")

    def _compressed_image_callback(self, msg: CompressedImage_):
        try:
            data_bytes = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            img_np = np.frombuffer(data_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
            if frame is not None:
                self.cache["/camera/image/compressed"] = frame
        except Exception as e:
            print(f"[RobotisDDSSDK] Compressed image decode error: {e}")

    def _image_callback(self, msg: Image_):
        try:
            img_data = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            frame = np.frombuffer(img_data, dtype=np.uint8)
            if msg.encoding == "mono8":
                frame = frame.reshape((msg.height, msg.width))
            elif msg.encoding in ["bgr8", "rgb8"]:
                frame = frame.reshape((msg.height, msg.width, 3))
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                print(f"[RobotisDDSSDK] Unsupported encoding: {msg.encoding}")
                return
            self.cache["/camera/image"] = frame
        except Exception as e:
            print(f"[RobotisDDSSDK] Image decode error: {e}")

    def _odom_callback(self, msg: Odometry_):
        self.cache["/odom"] = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": msg.pose.pose.orientation.z,
            "linear_vel": msg.twist.twist.linear.x,
            "angular_vel": msg.twist.twist.angular.z,
        }

    def _joint_state_callback(self, msg: JointState_):
        self.cache["/joint_states"] = dict(zip(msg.name, msg.position))

    def _battery_callback(self, msg: BatteryState_):
        self.cache["/battery_state"] = {
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        }

    def get(self, topic_name: str):
        self._ensure_subscription(topic_name)
        return self.cache.get(topic_name)

    def get_image(self): return self.get("/camera/image")
    def get_rgb_image(self): return self.get("/camera/image/compressed")
    def get_odometry(self): return self.get("/odom")
    def get_joint_state(self): return self.get("/joint_states")
    def get_battery_state(self): return self.get("/battery_state")

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist_(
            linear=Vector3_(x=linear_x, y=0.0, z=0.0),
            angular=Vector3_(x=0.0, y=0.0, z=angular_z),
        )
        self.cmd_vel_pub.publish(msg)

    def send_joint_trajectory(self, positions: list[float]):
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        header = Header_(stamp=Time_(sec=sec, nanosec=nsec), frame_id="base_link")
        point = JointTrajectoryPoint_(
            positions=positions,
            velocities=[],
            accelerations=[],
            effort=[],
            time_from_start=Time_(sec=1, nanosec=0),
        )
        msg = JointTrajectory_(
            header=header,
            joint_names=[f"joint_{i+1}" for i in range(len(positions))],
            points=[point],
        )
        self.joint_traj_pub.publish(msg)

    def load_policy(self, policy_path: str, policy_type: str = "GR00T_N1_5"):
        try:
            from gr00t.experiment.data_config import load_data_config
            from gr00t.model.policy import Gr00tPolicy
            data_config = load_data_config("aiworker_v1")
            if policy_type != "GR00T_N1_5":
                raise ValueError(f"Unsupported policy_type: {policy_type}")
            self.policy = Gr00tPolicy(
                model_path=policy_path,
                modality_config=data_config.modality_config(),
                modality_transform=data_config.transform(),
                embodiment_tag="new_embodiment",
                denoising_steps=4,
            )
            print(f"[RobotisDDSSDK] Policy loaded: {policy_path}")
        except Exception as e:
            print(f"[RobotisDDSSDK] Policy load failed: {e}")
            self.policy = None

    def unload_policy(self):
        self.policy = None
        torch.cuda.empty_cache()
        print("[RobotisDDSSDK] Policy unloaded")

    def get_action(self, data: dict):
        if self.policy is None:
            raise RuntimeError("Policy not loaded")
        result = self.policy.get_action(data)
        self._last_inference_result = result
        return result

    def process_request(self, cmd: str, payload: dict) -> dict:
        handlers = {
            "ping": self._handle_ping,
            "kill": self._handle_kill,
            "load_policy": self._handle_load_policy,
            "unload_policy": self._handle_unload_policy,
            "get_action": self._handle_get_action,
        }
        handler = handlers.get(cmd)
        if not handler:
            return {"status": "error", "message": f"Unknown command: {cmd}"}
        return handler(payload)

    def _handle_ping(self, data): return {"status": "ok", "message": "SDK alive"}
    def _handle_kill(self, data):
        self.unload_policy()
        return {"status": "ok", "message": "SDK stopped"}
    def _handle_load_policy(self, data):
        path = data.get("policy_path")
        if not path:
            return {"status": "error", "message": "policy_path required"}
        self.load_policy(path)
        return {"status": "ok", "message": "policy loaded"}
    def _handle_unload_policy(self, data):
        self.unload_policy()
        return {"status": "ok", "message": "policy unloaded"}
    def _handle_get_action(self, data):
        try:
            result = self.get_action(data)
            return {"status": "ok", "result": result}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _resolve_type(self, typename: str):
        import importlib
        mapping = {
            "physical_ai_interfaces::srv::dds_::Inference_Request_":
                "robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv._Inference_Request.Inference_Request",
            "physical_ai_interfaces::srv::dds_::Inference_Response_":
                "robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv._Inference_Response.Inference_Response",
        }
        if typename not in mapping:
            raise ValueError(f"Unknown DDS type: {typename}")
        module_path, class_name = mapping[typename].rsplit(".", 1)
        module = importlib.import_module(module_path)
        return getattr(module, class_name)

    def close(self):
        try:
            self.node.dds_destroy_node()
        except Exception:
            pass
        print("[RobotisDDSSDK] SDK closed.")


class DdsInferenceServer:
    def __init__(self, domain_id=30):
        self.sdk = RobotisDDSSDK(domain_id)
        self.running = True
        self._latest_req = None
        self.sdk.node.dds_create_subscription("/inference/request", Inference_Request, self._on_request)
        self.pub = self.sdk.node.dds_create_publisher("/inference/response", Inference_Response)
        print("[DdsInferenceServer] Initialized")

    def _on_request(self, msg):
        self._latest_req = msg

    def run(self):
        print("[DdsInferenceServer] Listening for /inference/request ...")
        try:
            while self.running:
                if not self._latest_req:
                    time.sleep(0.05)
                    continue

                req = self._latest_req
                self._latest_req = None

                try:
                    cmd = getattr(req, "command", "")
                    payload_bytes = bytes(req.payload)
                    data = torch.load(BytesIO(payload_bytes), weights_only=False)
                    result = self.sdk.process_request(cmd, data)
                except Exception as e:
                    result = {"status": "error", "message": str(e)}

                buf = BytesIO()
                torch.save(result, buf)
                resp_payload_list = list(buf.getvalue())

                msg = Inference_Response(
                    success=result.get("status") == "ok",
                    message=req.task_id,
                    payload=resp_payload_list,
                )
                self.pub.publish(msg)
        except KeyboardInterrupt:
            print("[DdsInferenceServer] Server stopped")
        finally:
            self.sdk.close()
