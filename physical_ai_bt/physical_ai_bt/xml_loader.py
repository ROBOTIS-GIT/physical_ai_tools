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

"""XML-based behavior tree loader for Groot compatibility."""

import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING, Dict, Type

from physical_ai_bt.actions import InferenceAction, RuleAction
from physical_ai_bt.actions.base_action import BTNode, BaseAction, BaseControl
from physical_ai_bt.controls import Sequence, Fallback

if TYPE_CHECKING:
    from rclpy.node import Node


class XMLTreeLoader:
    """Load behavior tree from XML file (Groot format)."""

    def __init__(self, node: 'Node', joint_names: list = None, runtime_params: Dict = None):
        """
        Initialize XML tree loader.

        Args:
            node: ROS2 node reference
            joint_names: Joint names from config (for RuleAction)
            runtime_params: Runtime parameters to inject into XML placeholders
        """
        self.node = node
        self.joint_names = joint_names or []
        self.runtime_params = runtime_params or {}

        # Register available node types
        self.control_types: Dict[str, Type[BaseControl]] = {
            'Sequence': Sequence,
            'Fallback': Fallback,
        }

        self.action_types: Dict[str, Type[BaseAction]] = {
            'InferenceAction': InferenceAction,
            'RuleAction': RuleAction,
        }

    def load_tree_from_file(self, xml_path: str, main_tree_id: str = None) -> BTNode:
        """
        Load behavior tree from XML file.

        Args:
            xml_path: Path to XML file
            main_tree_id: ID of main tree to execute (optional, reads from root)

        Returns:
            BTNode: Root node of the loaded tree
        """
        tree = ET.parse(xml_path)
        root = tree.getroot()

        # Get main tree ID
        if main_tree_id is None:
            main_tree_id = root.get('main_tree_to_execute')
            if not main_tree_id:
                raise ValueError("No main_tree_to_execute specified in XML")

        # Find the BehaviorTree with matching ID
        for behavior_tree in root.findall('BehaviorTree'):
            if behavior_tree.get('ID') == main_tree_id:
                # Load the first child (root node)
                return self._load_node(behavior_tree[0])

        raise ValueError(f"BehaviorTree with ID '{main_tree_id}' not found")

    def _load_node(self, xml_node: ET.Element) -> BTNode:
        """
        Recursively load a node from XML element.

        Args:
            xml_node: XML element representing a node

        Returns:
            BTNode: Loaded behavior tree node
        """
        node_type = xml_node.tag
        node_id = xml_node.get('ID', node_type)
        node_name = xml_node.get('name', node_id)

        # Load Control nodes
        if node_type in self.control_types:
            control_class = self.control_types[node_type]
            control_node = control_class(self.node, name=node_name)

            # Load children
            for child_xml in xml_node:
                child_node = self._load_node(child_xml)
                control_node.add_child(child_node)

            return control_node

        # Load Action nodes
        elif node_id in self.action_types:
            action_class = self.action_types[node_id]
            params = self._parse_node_params(xml_node)
            return self._create_action(action_class, node_name, params)

        else:
            raise ValueError(f"Unknown node type or ID: {node_type} / {node_id}")

    def _parse_node_params(self, xml_node: ET.Element) -> Dict:
        """
        Parse node parameters from XML attributes.

        Args:
            xml_node: XML element

        Returns:
            Dict: Parsed parameters with runtime injection
        """
        params = {}

        # Parse all attributes except ID and name
        for key, value in xml_node.attrib.items():
            if key not in ['ID', 'name']:
                # Resolve placeholders like {param_name}
                resolved_value = self._resolve_placeholder(value)
                # Try to convert to appropriate type
                params[key] = self._convert_value(resolved_value)

        return params

    def _resolve_placeholder(self, value: str) -> str:
        """
        Resolve placeholder values from runtime parameters.
        
        Args:
            value: String that may contain {placeholder}
            
        Returns:
            Resolved string value
        """
        # Check if value is a placeholder like {param_name}
        if isinstance(value, str) and value.startswith('{') and value.endswith('}'):
            param_name = value[1:-1]  # Remove { }
            if param_name in self.runtime_params:
                return str(self.runtime_params[param_name])
            else:
                self.node.get_logger().warn(
                    f"Placeholder '{value}' not found in runtime_params, using as-is"
                )
        return value

    def _convert_value(self, value: str):
        """Convert string value to appropriate type."""
        # Try float
        try:
            if '.' in value:
                return float(value)
            return int(value)
        except ValueError:
            pass

        # Try list (comma-separated)
        if ',' in value:
            parts = [p.strip() for p in value.split(',')]
            # Try to convert list elements
            try:
                return [float(p) if '.' in p else int(p) for p in parts]
            except ValueError:
                return parts  # Return as string list

        # Return as string
        return value

    def _create_action(self, action_class: Type[BaseAction], name: str, params: Dict) -> BaseAction:
        """
        Create action instance with parameters.

        Args:
            action_class: Action class to instantiate
            name: Node name
            params: Parameters dictionary

        Returns:
            BaseAction: Created action instance
        """
        # Map XML parameters to constructor arguments
        if action_class == InferenceAction:
            return InferenceAction(
                node=self.node,
                timeout=params.get('timeout', 5.0)
            )

        elif action_class == RuleAction:
            return RuleAction(
                node=self.node,
                current_positions=params.get('current_positions', []),
                target_positions=params.get('target_positions', []),
                joint_names=self.joint_names,  # Use joint_names from loader
                position_threshold=params.get('position_threshold', 0.1),
                timeout=params.get('timeout', 15.0)
            )

        else:
            raise ValueError(f"Unknown action class: {action_class}")
