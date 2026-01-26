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
# Author: Physical AI Team

from dataclasses import dataclass
import json
import logging
import os
from typing import Any, Callable, Dict, Optional

try:
    from zenoh_ros2_sdk import ROS2ServiceClient, ROS2Subscriber
except ImportError:
    ROS2ServiceClient = None
    ROS2Subscriber = None

logger = logging.getLogger(__name__)


@dataclass
class LeRobotResponse:
    success: bool
    message: str
    data: Dict[str, Any]
    request_id: str

    @classmethod
    def from_service_response(
        cls, response: Any, request_id: str = ''
    ) -> 'LeRobotResponse':
        if response is None:
            return cls(
                success=False,
                message='No response from service (timeout or error)',
                data={},
                request_id=request_id
            )
        return cls(
            success=getattr(response, 'success', False),
            message=getattr(response, 'message', ''),
            data=cls._extract_data(response),
            request_id=request_id
        )

    @staticmethod
    def _extract_data(response: Any) -> Dict[str, Any]:
        data = {}
        if hasattr(response, 'job_id'):
            data['job_id'] = response.job_id
        if hasattr(response, 'state'):
            data['state'] = response.state
        if hasattr(response, 'step'):
            data['step'] = response.step
        if hasattr(response, 'total_steps'):
            data['total_steps'] = response.total_steps
        if hasattr(response, 'loss'):
            data['loss'] = response.loss
        if hasattr(response, 'learning_rate'):
            data['learning_rate'] = response.learning_rate
        if hasattr(response, 'policies_json'):
            try:
                data['policies'] = json.loads(response.policies_json)
            except (json.JSONDecodeError, TypeError):
                data['policies_json'] = response.policies_json
        if hasattr(response, 'checkpoints_json'):
            try:
                data['checkpoints'] = json.loads(response.checkpoints_json)
            except (json.JSONDecodeError, TypeError):
                data['checkpoints_json'] = response.checkpoints_json
        if hasattr(response, 'models_json'):
            try:
                data['models'] = json.loads(response.models_json)
            except (json.JSONDecodeError, TypeError):
                data['models_json'] = response.models_json
        return data


class ZenohLeRobotClient:
    """
    Client for communicating with LeRobot Docker container via Zenoh ROS2 services.

    Uses zenoh_ros2_sdk's ROS2ServiceClient to call ROS2 services exposed by
    the LeRobot executor running in the Docker container.
    """

    SERVICE_TRAIN = '/lerobot/train'
    SERVICE_INFER = '/lerobot/infer'
    SERVICE_STOP = '/lerobot/stop'
    SERVICE_STATUS = '/lerobot/status'
    SERVICE_POLICY_LIST = '/lerobot/policy_list'
    SERVICE_CHECKPOINT_LIST = '/lerobot/checkpoint_list'
    SERVICE_MODEL_LIST = '/lerobot/model_list'

    TOPIC_PROGRESS = '/lerobot/progress'
    TOPIC_ACTION = '/lerobot/action'

    def __init__(
        self,
        router_ip: str = None,
        router_port: int = None,
        timeout_sec: float = 30.0
    ):
        self.router_ip = router_ip or os.environ.get('ZENOH_ROUTER_IP', '127.0.0.1')
        self.router_port = router_port or int(
            os.environ.get('ZENOH_ROUTER_PORT', '7447')
        )
        self.timeout_sec = timeout_sec

        self._connected = False
        self._service_clients: Dict[str, ROS2ServiceClient] = {}
        self._subscribers: list = []

        self._status_callback: Optional[Callable] = None
        self._action_callback: Optional[Callable] = None
        self._training_log_callback: Optional[Callable] = None

    def connect(self) -> bool:
        if ROS2ServiceClient is None:
            logger.warning('zenoh_ros2_sdk not available')
            return False

        try:
            self._init_service_clients()
            self._connected = True
            logger.info(
                f'Connected to Zenoh router at {self.router_ip}:{self.router_port}'
            )
            return True
        except Exception as e:
            logger.error(f'Connection failed: {e}')
            return False

    def _init_service_clients(self):
        service_configs = [
            (self.SERVICE_TRAIN, 'physical_ai_interfaces/srv/TrainModel'),
            (self.SERVICE_INFER, 'physical_ai_interfaces/srv/StartInference'),
            (self.SERVICE_STOP, 'physical_ai_interfaces/srv/StopTraining'),
            (self.SERVICE_STATUS, 'physical_ai_interfaces/srv/TrainingStatus'),
            (self.SERVICE_POLICY_LIST, 'physical_ai_interfaces/srv/PolicyList'),
            (self.SERVICE_CHECKPOINT_LIST, 'physical_ai_interfaces/srv/CheckpointList'),
            (self.SERVICE_MODEL_LIST, 'physical_ai_interfaces/srv/ModelList'),
        ]

        for service_name, srv_type in service_configs:
            try:
                client = ROS2ServiceClient(
                    service_name=service_name,
                    srv_type=srv_type,
                    router_ip=self.router_ip,
                    router_port=self.router_port,
                    timeout=self.timeout_sec
                )
                self._service_clients[service_name] = client
            except Exception as e:
                logger.warning(f'Failed to create client for {service_name}: {e}')

    def disconnect(self):
        for sub in self._subscribers:
            try:
                sub.close()
            except Exception:
                pass
        self._subscribers.clear()

        for name, client in self._service_clients.items():
            try:
                client.close()
            except Exception:
                pass
        self._service_clients.clear()

        self._connected = False

    def _call_service(self, service_name: str, **kwargs) -> LeRobotResponse:
        if not self._connected:
            return LeRobotResponse(
                success=False,
                message='Not connected to Zenoh',
                data={},
                request_id=''
            )

        client = self._service_clients.get(service_name)
        if client is None:
            return LeRobotResponse(
                success=False,
                message=f'Service client not found: {service_name}',
                data={},
                request_id=''
            )

        try:
            response = client.call(**kwargs)
            return LeRobotResponse.from_service_response(response)
        except Exception as e:
            logger.error(f'Service call to {service_name} failed: {e}')
            return LeRobotResponse(
                success=False,
                message=f'Service call failed: {str(e)}',
                data={},
                request_id=''
            )

    def start_training(
        self,
        policy_type: str,
        dataset_path: str,
        output_dir: str = None,
        num_epochs: int = None,
        batch_size: int = None,
        learning_rate: float = None,
        eval_freq: int = None,
        log_freq: int = None,
        save_freq: int = None,
        wandb_project: str = None
    ) -> LeRobotResponse:
        kwargs = {
            'policy_type': policy_type,
            'dataset_path': dataset_path,
            'output_dir': output_dir or '',
            'steps': num_epochs or 0,
            'batch_size': batch_size or 0,
            'learning_rate': learning_rate or 0.0,
            'eval_freq': eval_freq or 0,
            'log_freq': log_freq or 0,
            'save_freq': save_freq or 0,
            'wandb_project': wandb_project or '',
            'push_to_hub': False
        }
        return self._call_service(self.SERVICE_TRAIN, **kwargs)

    def stop_training(self) -> LeRobotResponse:
        return self._call_service(self.SERVICE_STOP)

    def resume_training(self, checkpoint_path: str) -> LeRobotResponse:
        return self._call_service(
            self.SERVICE_TRAIN,
            policy_type='',
            dataset_path='',
            output_dir=checkpoint_path,
            steps=0,
            batch_size=0,
            learning_rate=0.0,
            save_freq=0,
            wandb_project='',
            push_to_hub=False
        )

    def get_training_status(self) -> LeRobotResponse:
        return self._call_service(self.SERVICE_STATUS)

    def start_inference(
        self,
        model_path: str,
        env_name: str = None,
        num_episodes: int = None
    ) -> LeRobotResponse:
        kwargs = {
            'model_path': model_path,
            'image_topics': [],
            'joint_state_topic': ''
        }
        return self._call_service(self.SERVICE_INFER, **kwargs)

    def stop_inference(self) -> LeRobotResponse:
        return self._call_service(self.SERVICE_STOP)

    def get_inference_status(self) -> LeRobotResponse:
        return self._call_service(self.SERVICE_STATUS)

    def load_model(self, model_path: str) -> LeRobotResponse:
        return self.start_inference(model_path)

    def unload_model(self) -> LeRobotResponse:
        return self.stop_inference()

    def list_models(self) -> LeRobotResponse:
        return self._call_service(self.SERVICE_MODEL_LIST)

    def subscribe_status(self, callback: Callable[[Dict], None]) -> bool:
        """Subscribe to training progress updates from LeRobot executor."""
        if ROS2Subscriber is None:
            logger.warning('ROS2Subscriber not available')
            return False

        self._status_callback = callback

        try:
            def on_progress(msg):
                """Handle TrainingProgress message from executor."""
                data = {
                    'status': getattr(msg, 'state', 'unknown'),
                    'step': getattr(msg, 'step', 0),
                    'total_steps': getattr(msg, 'total_steps', 0),
                    'loss': getattr(msg, 'loss', 0.0),
                    'learning_rate': getattr(msg, 'learning_rate', 0.0),
                    'gradient_norm': getattr(msg, 'gradient_norm', 0.0),
                    'elapsed_seconds': getattr(msg, 'elapsed_seconds', 0.0),
                    'eta_seconds': getattr(msg, 'eta_seconds', 0.0),
                }
                callback(data)

            sub = ROS2Subscriber(
                topic=self.TOPIC_PROGRESS,
                msg_type='physical_ai_interfaces/msg/TrainingProgress',
                callback=on_progress,
                router_ip=self.router_ip,
                router_port=self.router_port
            )
            self._subscribers.append(sub)
            return True
        except Exception as e:
            logger.error(f'Failed to subscribe to progress: {e}')
            return False

    def subscribe_actions(self, callback: Callable[[Dict], None]) -> bool:
        """Subscribe to action outputs from inference."""
        if ROS2Subscriber is None:
            return False

        self._action_callback = callback

        try:
            def on_action(msg):
                data = {
                    'seq': getattr(msg, 'seq', 0),
                    'joint_positions': list(getattr(msg, 'joint_positions', [])),
                    'gripper': getattr(msg, 'gripper', 0.0),
                    'timestamp': getattr(msg, 'timestamp', ''),
                }
                callback(data)

            sub = ROS2Subscriber(
                topic=self.TOPIC_ACTION,
                msg_type='physical_ai_interfaces/msg/ActionOutput',
                callback=on_action,
                router_ip=self.router_ip,
                router_port=self.router_port
            )
            self._subscribers.append(sub)
            return True
        except Exception as e:
            logger.error(f'Failed to subscribe to actions: {e}')
            return False

    def subscribe_training_log(self, callback: Callable[[Dict], None]) -> bool:
        """Subscribe to training log - uses same progress topic."""
        self._training_log_callback = callback
        return True

    def list_checkpoints(self) -> LeRobotResponse:
        return self._call_service(self.SERVICE_CHECKPOINT_LIST)

    def get_checkpoint_info(self, checkpoint_path: str) -> LeRobotResponse:
        return LeRobotResponse(
            success=False,
            message='Not implemented via ROS2 service',
            data={},
            request_id=''
        )

    def delete_checkpoint(self, checkpoint_path: str) -> LeRobotResponse:
        return LeRobotResponse(
            success=False,
            message='Not implemented via ROS2 service',
            data={},
            request_id=''
        )

    def get_policy_list(self, category: str = None) -> LeRobotResponse:
        kwargs = {'category': category or ''}
        return self._call_service(self.SERVICE_POLICY_LIST, **kwargs)
