#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
# Author: Seongwoo Kim

"""Action nodes for docking system control via SetBool services."""

import time
from typing import TYPE_CHECKING

from example_interfaces.srv import AddTwoInts
from physical_ai_bt.actions.base_action import BaseAction
from physical_ai_bt.bt_core import NodeStatus
from std_srvs.srv import SetBool

if TYPE_CHECKING:
    from rclpy.node import Node

SERVICE_CALL_TIMEOUT_SEC = 30.0


class _SetBoolAction(BaseAction):
    """Base action that calls a SetBool service."""

    _STATE_INIT = 'init'
    _STATE_WAITING_SERVICE = 'waiting_service'
    _STATE_CALLING = 'calling'
    _STATE_DONE = 'done'

    def __init__(
        self,
        node: 'Node',
        service_name: str,
        data: bool = True,
        name: str = 'SetBoolAction',
    ):
        super().__init__(node, name=name)
        self._data = data
        self._client = self.node.create_client(SetBool, service_name)
        self._state = self._STATE_INIT
        self._future = None
        self._result = None
        self._start_time = None

    def tick(self) -> NodeStatus:
        if self._state == self._STATE_INIT:
            self.log_info(
                f'{self.name} started (data={self._data})'
            )
            self._state = self._STATE_WAITING_SERVICE
            self._start_time = time.monotonic()
            return NodeStatus.RUNNING

        if self._state == self._STATE_WAITING_SERVICE:
            if not self._client.service_is_ready():
                if time.monotonic() - self._start_time > SERVICE_CALL_TIMEOUT_SEC:
                    self.log_error(f'{self.name}: service not available')
                    self._state = self._STATE_DONE
                    self._result = False
                    return NodeStatus.FAILURE
                return NodeStatus.RUNNING

            req = SetBool.Request()
            req.data = self._data
            self._future = self._client.call_async(req)
            self._start_time = time.monotonic()
            self._state = self._STATE_CALLING
            return NodeStatus.RUNNING

        if self._state == self._STATE_CALLING:
            if not self._future.done():
                if time.monotonic() - self._start_time > SERVICE_CALL_TIMEOUT_SEC:
                    self.log_error(f'{self.name}: service call timed out')
                    self._future.cancel()
                    self._state = self._STATE_DONE
                    self._result = False
                    return NodeStatus.FAILURE
                return NodeStatus.RUNNING

            response = self._future.result()
            if response is None or not response.success:
                msg = response.message if response else 'No response'
                self.log_error(f'{self.name} failed: {msg}')
                self._state = self._STATE_DONE
                self._result = False
                return NodeStatus.FAILURE

            self.log_info(f'{self.name}: {response.message}')
            self._state = self._STATE_DONE
            self._result = True
            return NodeStatus.SUCCESS

        # _STATE_DONE
        return NodeStatus.SUCCESS if self._result else NodeStatus.FAILURE

    def reset(self):
        super().reset()
        if self._future is not None and not self._future.done():
            self._future.cancel()
        self._future = None
        self._state = self._STATE_INIT
        self._result = None
        self._start_time = None


class DockingPerception(_SetBoolAction):
    """Activate or deactivate docking perception."""

    def __init__(
        self,
        node: 'Node',
        activate: bool = True,
    ):
        super().__init__(
            node=node,
            service_name='/docking_perception_activate',
            data=activate,
            name='DockingPerception',
        )


class MarkerMove(_SetBoolAction):
    """Move to docking position A (true) or B (false)."""

    def __init__(
        self,
        node: 'Node',
        move_to_a: bool = True,
    ):
        super().__init__(
            node=node,
            service_name='/marker_ab_move',
            data=move_to_a,
            name='MarkerMove',
        )


class MoveAbMove(BaseAction):
    """Call /move_ab_move AddTwoInts service with configurable a and b values."""

    _STATE_INIT = 'init'
    _STATE_WAITING_SERVICE = 'waiting_service'
    _STATE_CALLING = 'calling'
    _STATE_DONE = 'done'

    def __init__(
        self,
        node: 'Node',
        a: int = 1,
        b: int = 0,
    ):
        super().__init__(node, name='MoveAbMove')
        self._a = int(a)
        self._b = int(b)
        self._client = self.node.create_client(AddTwoInts, '/move_ab_move')
        self._state = self._STATE_INIT
        self._future = None
        self._result = None
        self._start_time = None

    def tick(self) -> NodeStatus:
        if self._state == self._STATE_INIT:
            self.log_info(
                f'{self.name} started (a={self._a}, b={self._b})'
            )
            self._state = self._STATE_WAITING_SERVICE
            self._start_time = time.monotonic()
            return NodeStatus.RUNNING

        if self._state == self._STATE_WAITING_SERVICE:
            if not self._client.service_is_ready():
                if time.monotonic() - self._start_time > SERVICE_CALL_TIMEOUT_SEC:
                    self.log_error(f'{self.name}: service not available')
                    self._state = self._STATE_DONE
                    self._result = False
                    return NodeStatus.FAILURE
                return NodeStatus.RUNNING

            req = AddTwoInts.Request()
            req.a = self._a
            req.b = self._b
            self._future = self._client.call_async(req)
            self._start_time = time.monotonic()
            self._state = self._STATE_CALLING
            return NodeStatus.RUNNING

        if self._state == self._STATE_CALLING:
            if not self._future.done():
                if time.monotonic() - self._start_time > SERVICE_CALL_TIMEOUT_SEC:
                    self.log_error(f'{self.name}: service call timed out')
                    self._future.cancel()
                    self._state = self._STATE_DONE
                    self._result = False
                    return NodeStatus.FAILURE
                return NodeStatus.RUNNING

            response = self._future.result()
            if response is None:
                self.log_error(f'{self.name} failed: No response')
                self._state = self._STATE_DONE
                self._result = False
                return NodeStatus.FAILURE

            self.log_info(f'{self.name}: result={response.sum}')
            self._state = self._STATE_DONE
            self._result = True
            return NodeStatus.SUCCESS

        # _STATE_DONE
        return NodeStatus.SUCCESS if self._result else NodeStatus.FAILURE

    def reset(self):
        super().reset()
        if self._future is not None and not self._future.done():
            self._future.cancel()
        self._future = None
        self._state = self._STATE_INIT
        self._result = None
        self._start_time = None
