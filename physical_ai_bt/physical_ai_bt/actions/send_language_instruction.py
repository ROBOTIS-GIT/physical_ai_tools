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
# Author: Seongwoo Kim

from physical_ai_bt.actions.base_action import NodeStatus, BaseAction
from physical_ai_interfaces.srv import SendCommand

class SendLanguageInstruction(BaseAction):
    def __init__(self, node, instruction):
        super().__init__(node, name="SendLanguageInstruction")
        self.instruction = instruction
        self.client = self.node.create_client(SendCommand, '/task/command')
        self.sent = False
        self.future = None

    def tick(self) -> NodeStatus:
        if not self.sent:
            if not self.client.wait_for_service(timeout_sec=2.0):
                return NodeStatus.FAILURE
            req = SendCommand.Request()
            req.command = SendCommand.Request.START_INFERENCE
            if hasattr(self.node, 'latest_status') and self.node.latest_status is not None:
                import copy
                req.task_info = copy.deepcopy(self.node.latest_status.task_info)
                req.task_info.task_instruction = [self.instruction]
            else:
                from physical_ai_interfaces.msg import TaskInfo
                req.task_info = TaskInfo()
                req.task_info.task_instruction = [self.instruction]
            self.future = self.client.call_async(req)
            self.sent = True
            return NodeStatus.RUNNING
        if self.future and self.future.done():
            result = self.future.result()
            self.log_info(f"SendLanguageInstruction response: success={result.success}, message={getattr(result, 'message', '')}")
            if result.success:
                return NodeStatus.SUCCESS
            else:
                return NodeStatus.FAILURE
        return NodeStatus.RUNNING
