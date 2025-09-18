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

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple
from rclpy.node import Node


class BaseServiceManager(ABC):
    """
    Base class for all service managers.
    
    This class provides a common interface and structure for managing
    groups of related ROS services in the Physical AI Server.
    """
    
    def __init__(self, node: Node):
        """
        Initialize the service manager.
        
        Args:
            node: ROS2 node instance for creating services and logging
        """
        self.node = node
        self.logger = node.get_logger()
        self._services: List[Tuple[str, Any, Any]] = []
        
    def register_service(self, service_name: str, service_type: Any, 
                        callback: Any) -> None:
        """
        Register a ROS service with the node.
        
        Args:
            service_name: Name of the service
            service_type: Type of the service
            callback: Callback function for the service
        """
        try:
            service = self.node.create_service(service_type, service_name, callback)
            self._services.append((service_name, service_type, service))
            self.logger.info(f'Service registered: {service_name}')
        except Exception as e:
            self.logger.error(f'Failed to register service {service_name}: {str(e)}')
            raise
    
    def register_services(self, service_definitions: List[Tuple[str, Any, Any]]) -> None:
        """
        Register multiple services at once.
        
        Args:
            service_definitions: List of (service_name, service_type, callback) tuples
        """
        for service_name, service_type, callback in service_definitions:
            self.register_service(service_name, service_type, callback)
    
    @abstractmethod
    def initialize_services(self) -> None:
        """
        Initialize all services managed by this manager.
        This method should be implemented by subclasses to register
        their specific services.
        """
        pass
    
    def cleanup(self) -> None:
        """
        Cleanup resources used by this service manager.
        Override in subclasses if specific cleanup is needed.
        """
        self.logger.info(f'Cleaning up {self.__class__.__name__}')
        # Services are automatically cleaned up when the node is destroyed
        self._services.clear()
    
    def get_service_count(self) -> int:
        """
        Get the number of services managed by this manager.
        
        Returns:
            Number of registered services
        """
        return len(self._services)
    
    def get_service_names(self) -> List[str]:
        """
        Get the names of all services managed by this manager.
        
        Returns:
            List of service names
        """
        return [service_name for service_name, _, _ in self._services]
