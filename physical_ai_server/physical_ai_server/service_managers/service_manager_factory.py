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

from typing import Any, Dict, List

from physical_ai_server.service_managers.base_service_manager import BaseServiceManager
from physical_ai_server.service_managers.data_collection_service_manager \
    import DataCollectionServiceManager
from physical_ai_server.service_managers.edit_data_service_manager \
    import EditDataServiceManager
from physical_ai_server.service_managers.huggingface_service_manager \
    import HuggingFaceServiceManager
from physical_ai_server.service_managers.inference_service_manager import InferenceServiceManager
from physical_ai_server.service_managers.system_info_service_manager \
    import SystemInfoServiceManager
from physical_ai_server.service_managers.training_service_manager import TrainingServiceManager

from rclpy.node import Node


class ServiceManagerFactory:
    """
    Factory class for creating and managing service managers.

    This class provides a centralized way to create and initialize
    all service managers for the Physical AI Server.
    """

    def __init__(self, node: Node, main_server: Any):
        """
        Initialize the service manager factory.

        Args:
            node: ROS2 node instance
            main_server: Reference to the main PhysicalAIServer instance
        """
        self.node = node
        self.main_server = main_server
        self.service_managers: Dict[str, BaseServiceManager] = {}

    def create_all_managers(self) -> Dict[str, BaseServiceManager]:
        """
        Create and initialize all service managers.

        Returns:
            Dictionary of service manager names to manager instances
        """
        self.logger = self.node.get_logger()
        self.logger.info('Creating all service managers...')

        # Create service managers
        managers = {
            'data_collection': DataCollectionServiceManager(self.node, self.main_server),
            'inference': InferenceServiceManager(self.node, self.main_server),
            'training': TrainingServiceManager(self.node, self.main_server),
            'edit_data': EditDataServiceManager(self.node, self.main_server),
            'huggingface': HuggingFaceServiceManager(self.node, self.main_server),
            'system_info': SystemInfoServiceManager(self.node, self.main_server),
        }

        # Initialize all managers
        for name, manager in managers.items():
            try:
                manager.initialize_services()
                self.service_managers[name] = manager
                self.logger.info(f'Service manager "{name}" created successfully')
            except Exception as e:
                self.logger.error(f'Failed to create service manager "{name}": {str(e)}')
                raise

        self.logger.info(f'All {len(managers)} service managers created successfully')
        return self.service_managers

    def get_manager(self, name: str) -> BaseServiceManager:
        """
        Get a specific service manager by name.

        Args:
            name: Name of the service manager

        Returns:
            Service manager instance

        Raises:
            KeyError: If the manager name is not found
        """
        if name not in self.service_managers:
            raise KeyError(f'Service manager "{name}" not found')
        return self.service_managers[name]

    def get_all_managers(self) -> Dict[str, BaseServiceManager]:
        """
        Get all service managers.

        Returns:
            Dictionary of all service managers
        """
        return self.service_managers.copy()

    def cleanup_all_managers(self) -> None:
        """
        Cleanup all service managers.
        """
        self.logger.info('Cleaning up all service managers...')
        for name, manager in self.service_managers.items():
            try:
                manager.cleanup()
                self.logger.info(f'Service manager "{name}" cleaned up successfully')
            except Exception as e:
                self.logger.error(f'Error cleaning up service manager "{name}": {str(e)}')

        self.service_managers.clear()
        self.logger.info('All service managers cleaned up')

    def get_total_service_count(self) -> int:
        """
        Get the total number of services across all managers.

        Returns:
            Total number of services
        """
        return sum(manager.get_service_count() for manager in self.service_managers.values())

    def get_service_summary(self) -> Dict[str, List[str]]:
        """
        Get a summary of all services by manager.

        Returns:
            Dictionary mapping manager names to their service names
        """
        summary = {}
        for name, manager in self.service_managers.items():
            summary[name] = manager.get_service_names()
        return summary
