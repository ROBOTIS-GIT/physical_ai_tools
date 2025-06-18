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

from dataclasses import dataclass
from io import BytesIO
from typing import Callable

from inference_manager import InferenceManager
import torch
import zmq


class TorchSerializer:

    @staticmethod
    def to_bytes(data: dict) -> bytes:
        buffer = BytesIO()
        torch.save(data, buffer)
        return buffer.getvalue()

    @staticmethod
    def from_bytes(data: bytes) -> dict:
        buffer = BytesIO(data)
        obj = torch.load(buffer, weights_only=False)
        return obj


@dataclass
class EndpointHandler:

    handler: Callable
    requires_input: bool = True


class ServerInference:

    def __init__(
            self,
            policy_type: str,
            policy_path: str,
            device: str,
            server_address: str,
            port: int = 5555):

        self.inference_manager = InferenceManager(
            policy_type=policy_type,
            policy_path=policy_path,
            device=device
        )

        self.running = True
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f'tcp://{server_address}:{port}')

        self.inference_manager = InferenceManager(
            policy_type=policy_type,
            policy_path=policy_path,
            device=device
        )

        # Register the ping endpoint by default
        self.register_endpoint('ping', self._handle_ping, requires_input=False)
        self.register_endpoint('kill', self._kill_server, requires_input=False)
        self.register_endpoint('get_action', self.inference_manager.predict)
        self.register_endpoint(
            'get_modality_config',
            self.inference_manager.get_policy_config,
            requires_input=False
        )

    def _kill_server(self):
        self.running = False

    def _handle_ping(self) -> dict:
        return {'status': 'ok', 'message': 'Server is running'}

    def register_endpoint(
            self,
            name: str,
            handler: Callable,
            requires_input: bool = True):
        self._endpoints[name] = EndpointHandler(handler, requires_input)

    def run(self):
        addr = self.socket.getsockopt_string(zmq.LAST_ENDPOINT)
        print(f'Server is ready and listening on {addr}')
        while self.running:
            try:
                message = self.socket.recv()
                request = TorchSerializer.from_bytes(message)
                endpoint = request.get('endpoint', 'get_action')

                if endpoint not in self._endpoints:
                    raise ValueError(f'Unknown endpoint: {endpoint}')

                handler = self._endpoints[endpoint]
                result = (
                    handler.handler(request.get('data', {}))
                    if handler.requires_input
                    else handler.handler()
                )
                self.socket.send(TorchSerializer.to_bytes(result))
            except Exception as e:
                print(f'Error in server: {e}')
                import traceback

                print(traceback.format_exc())
                self.socket.send(b'ERROR')
