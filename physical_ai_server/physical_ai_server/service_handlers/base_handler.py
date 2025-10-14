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
# Author: Kiwoong Park


class BaseServiceHandler:
    """
    Base class for all service handlers.

    Provides common functionality and helper methods for service callbacks.
    """

    def __init__(self, node):
        """
        Initialize base service handler.

        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.logger = node.get_logger()

    def _create_error_response(self, response, message):
        """
        Create an error response with standard format.

        Args:
            response: Service response object
            message: Error message string

        Returns:
            response: Modified response object with error status
        """
        response.success = False
        response.message = message
        return response

    def _create_success_response(self, response, message):
        """
        Create a success response with standard format.

        Args:
            response: Service response object
            message: Success message string

        Returns:
            response: Modified response object with success status
        """
        response.success = True
        response.message = message
        return response