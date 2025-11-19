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

"""
DDSNode - A lightweight ROS 2-like interface for DDS communication without ROS 2 dependencies.

This module provides a simple way to:
1. Communicate with ROS 2 systems without installing ROS 2
2. Use ROS 2 Node patterns (topics, services, actions) via DDS
3. Run in lightweight environments (Docker, AI servers, etc.)

Example:
    node = DDSNode(name='my_node')

    # Create subscriptions with callbacks
    node.dds_create_subscription('/topic1', Type1_, callback1)
    node.dds_create_subscription('/topic2', Type2_, callback2)

    # Create publishers
    pub = node.dds_create_publisher('/cmd_topic', CmdType_)
    pub.publish(msg)  # ROS 2 style

    # Spin to keep callbacks active
    node.dds_spin()
"""

from dataclasses import dataclass
import os
import time
from typing import Any, Callable, Dict, Type
import uuid

from cyclonedds.core import Listener, Policy, Qos
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration


class DDSPublisher:
    """
    Wrapper for CycloneDDS DataWriter with ROS 2-like interface.

    Provides publish() method for ROS 2 compatibility.
    """

    def __init__(self, writer: DataWriter):
        """Initialize publisher wrapper with a DataWriter."""
        self._writer = writer

    def publish(self, msg: Any) -> None:
        """
        Publish a message (ROS 2 style).

        :param msg: The message to publish
        """
        self._writer.write(msg)


class CallbackListener(Listener):
    """Listener that triggers a callback when data is available."""

    def __init__(self, callback: Callable):
        super().__init__()
        self.callback = callback

    def on_data_available(self, reader):
        """Process new data when available on the reader."""
        # Process all available samples
        while True:
            samples = reader.take(N=100)  # Take up to 100 samples at once
            if not samples:
                break
            for sample in samples:
                try:
                    # Skip invalid samples (e.g., disposal notifications)
                    if hasattr(sample, '__class__') and 'Invalid' in sample.__class__.__name__:
                        continue
                    self.callback(sample)
                except Exception as e:
                    print(f'Error in callback: {e}')


@dataclass
class ServiceRequest:
    """Container for service request with client info."""

    client_guid: bytes
    sequence_number: int
    request: Any


@dataclass
class ServiceResponse:
    """Container for service response."""

    client_guid: bytes
    sequence_number: int
    response: Any


class DDSNode:
    """
    A DDS-based Node that manages topics, services, and actions (ROS 2 compatible).

    A single DDSNode instance can handle multiple publishers, subscribers, services, and actions,
    making it easy to communicate with ROS 2 systems without ROS 2 dependencies.
    """

    def __init__(
        self,
        name: str = 'dds_node',
        domain_id: int = None,
        peers: list = None,
        network_interface: str = 'auto',
        allow_multicast: bool = True,
        port_base: int = 7400
    ):
        """
        Initialize DDSNode (similar to ROS 2 Node).

        :param name: Name of the DDS node (similar to ROS 2 node name)
        :param domain_id: DDS domain ID (defaults to ROS_DOMAIN_ID environment variable)
        :param peers: List of peer addresses for remote communication
            (e.g., ['192.168.1.100:7400', 'cloud.example.com'])
        :param network_interface: Network interface to use
            ('auto' or specific IP address like '192.168.1.100')
        :param allow_multicast: Enable multicast discovery (True for LAN, False for WAN)
        :param port_base: Base port for DDS discovery (default: 7400)

        Example:
            # Local network (default)
            node = DDSNode(name='local_node')

            # Connect to remote cloud server
            node = DDSNode(
                name='robot_node',
                peers=['cloud.example.com:7400', '52.12.34.56:7400'],
                allow_multicast=False
            )

            # Cloud server listening on all interfaces
            node = DDSNode(
                name='cloud_node',
                network_interface='0.0.0.0',
                allow_multicast=False,
                port_base=7400
            )
        """
        self.name = name
        if domain_id is None:
            try:
                domain_id = int(os.getenv('ROS_DOMAIN_ID', 0))
            except (ValueError, TypeError):
                domain_id = 0
        self.domain_id = domain_id

        # Configure network settings
        self._configure_network(peers, network_interface, allow_multicast, port_base)

        self.domain_participant = DomainParticipant(domain_id)

        # Store all subscribers and publishers
        self.subscribers: Dict[str, DataReader] = {}
        self.publishers: Dict[str, DataWriter] = {}

        # Store services and clients
        self.services: Dict[str, Dict] = {}
        self.clients: Dict[str, Dict] = {}

        self._running = False
        self._client_guid = uuid.uuid4().bytes[:16]  # Unique client ID

    def _configure_network(
        self, peers: list = None, network_interface: str = 'auto',
        allow_multicast: bool = True, port_base: int = 7400
    ):
        """
        Configure CycloneDDS network settings via CYCLONEDDS_URI environment variable.

        :param peers: List of peer addresses for remote communication
        :param network_interface: Network interface to use
        :param allow_multicast: Enable/disable multicast discovery
        :param port_base: Base port for DDS discovery
        """
        # Skip if CYCLONEDDS_URI is already set by user
        if os.getenv('CYCLONEDDS_URI'):
            return

        # Build XML configuration
        xml_config = ['<CycloneDDS><Domain>']

        # General settings
        xml_config.append('<General>')
        xml_config.append(
            f'<NetworkInterfaceAddress>{network_interface}</NetworkInterfaceAddress>'
        )
        xml_config.append(
            f'<AllowMulticast>{"true" if allow_multicast else "false"}</AllowMulticast>'
        )
        xml_config.append('</General>')

        # Discovery settings
        xml_config.append('<Discovery>')

        # Add peers if specified
        if peers:
            xml_config.append('<Peers>')
            for peer in peers:
                # Add default port if not specified
                if ':' not in peer:
                    peer = f'{peer}:{port_base}'
                xml_config.append(f'<Peer address="{peer}"/>')
            xml_config.append('</Peers>')

        # Set port base if different from default
        if port_base != 7400:
            xml_config.append('<Ports>')
            xml_config.append(f'<Base>{port_base}</Base>')
            xml_config.append('</Ports>')

        xml_config.append('</Discovery>')
        xml_config.append('</Domain></CycloneDDS>')

        # Set environment variable
        os.environ['CYCLONEDDS_URI'] = ''.join(xml_config)

        print(f'[{self.name}] Network configured:')
        print(f'  Interface: {network_interface}')
        print(f'  Multicast: {allow_multicast}')
        print(f'  Port base: {port_base}')
        if peers:
            print(f"  Peers: {', '.join(peers)}")

    def _normalize_topic_name(self, topic_name: str) -> str:
        """Normalize topic name by adding 'rt/' prefix if needed (ROS 2 DDS convention)."""
        if topic_name.startswith('rt/'):
            return topic_name
        elif topic_name.startswith('/'):
            return 'rt' + topic_name
        else:
            return 'rt/' + topic_name

    def dds_create_subscription(
        self, topic_name: str, topic_type: Type,
        callback: Callable, qos: Qos = None
    ) -> DataReader:
        """
        Create a DDS subscription (similar to ROS 2 Node.create_subscription).

        :param topic_name: The topic name for the subscription
        :param topic_type: The message type for the topic
        :param callback: Callback function that will be called when data arrives
        :param qos: Optional QoS settings for the subscription
        :return: A configured DataReader instance

        Example:
            def my_callback(msg):
                print(f'Received: {msg}')

            sub = node.dds_create_subscription('/joint_states', JointState_, my_callback)
        """
        if qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(duration()),
                Policy.Durability.Volatile,
                Policy.History.KeepLast(10)
            )

        listener = CallbackListener(callback)
        final_topic_name = self._normalize_topic_name(topic_name)

        topic = Topic(self.domain_participant, final_topic_name, topic_type, qos=qos)
        reader = DataReader(self.domain_participant, topic, listener=listener)

        # Store the subscriber
        self.subscribers[topic_name] = reader

        return reader

    def dds_create_publisher(
        self, topic_name: str, topic_type: Type, qos: Qos = None
    ) -> DDSPublisher:
        """
        Create a DDS publisher (similar to ROS 2 Node.create_publisher).

        :param topic_name: The topic name for the publisher
        :param topic_type: The message type for the topic
        :param qos: Optional QoS settings for the publisher
        :return: A DDSPublisher instance with publish() method

        Example:
            pub = node.dds_create_publisher('/cmd_vel', Twist_)
            pub.publish(twist_msg)
        """
        if qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(duration()),
                Policy.Durability.Volatile,
                Policy.History.KeepLast(10)
            )

        final_topic_name = self._normalize_topic_name(topic_name)

        topic = Topic(self.domain_participant, final_topic_name, topic_type, qos=qos)
        publisher = Publisher(self.domain_participant, qos=qos)
        writer = DataWriter(publisher, topic)

        # Wrap in DDSPublisher for ROS 2-like interface
        dds_pub = DDSPublisher(writer)

        # Store the publisher
        self.publishers[topic_name] = dds_pub

        return dds_pub

    def dds_create_service(
        self, service_name: str, request_type: Type, response_type: Type,
        callback: Callable, qos: Qos = None
    ):
        """
        Create a DDS service server (similar to ROS 2 Node.create_service).

        :param service_name: The service name
        :param request_type: The request message type
        :param response_type: The response message type
        :param callback: Callback function(request) -> response
        :param qos: Optional QoS settings
        :return: Service dictionary with reader and writer

        Example:
            def handle_set_robot_type(request):
                response = SetRobotType_Response()
                response.success = True
                response.message = f"Robot type set to {request.robot_type}"
                return response

            service = node.dds_create_service(
                '/set_robot_type',
                SetRobotType_Request,
                SetRobotType_Response,
                handle_set_robot_type
            )
        """
        if qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(duration()),
                Policy.Durability.Volatile,
                Policy.History.KeepLast(10)
            )

        # ROS 2 service naming convention
        # Request: rq/<service_name>Request
        # Response: rr/<service_name>Reply
        req_topic_name = f'rq{service_name}Request'
        res_topic_name = f'rr{service_name}Reply'

        # Create request reader (server receives requests)
        req_topic = Topic(self.domain_participant, req_topic_name, request_type, qos=qos)

        def service_callback(request):
            """Wrap user callback for internal service handling."""
            try:
                # Call user's service handler
                response = callback(request)

                # Send response back
                res_writer = self.services[service_name]['response_writer']
                res_writer.write(response)
            except Exception as e:
                print(f'Error in service callback: {e}')

        req_listener = CallbackListener(service_callback)
        req_reader = DataReader(self.domain_participant, req_topic, listener=req_listener)

        # Create response writer (server sends responses)
        res_topic = Topic(self.domain_participant, res_topic_name, response_type, qos=qos)
        res_publisher = Publisher(self.domain_participant, qos=qos)
        res_writer = DataWriter(res_publisher, res_topic)

        # Store service
        self.services[service_name] = {
            'request_reader': req_reader,
            'response_writer': res_writer,
            'request_type': request_type,
            'response_type': response_type
        }

        return self.services[service_name]

    def dds_create_client(
        self,
        service_name: str,
        request_type: Type,
        response_type: Type,
        qos: Qos = None
    ):
        """
        Create a DDS service client (similar to ROS 2 Node.create_client).

        :param service_name: The service name
        :param request_type: The request message type
        :param response_type: The response message type
        :param qos: Optional QoS settings
        :return: Client dictionary with reader and writer

        Example:
            client = node.dds_create_client(
                '/set_robot_type',
                SetRobotType_Request,
                SetRobotType_Response
            )

            # Make a request
            request = SetRobotType_Request(robot_type='ai_worker')
            response = node.dds_call_service(client, request, timeout=5.0)
            print(f'Success: {response.success}')
            print(f'Message: {response.message}')
        """
        if qos is None:
            qos = Qos(
                Policy.Reliability.Reliable(duration()),
                Policy.Durability.Volatile,
                Policy.History.KeepLast(10)
            )

        # ROS 2 service naming convention
        req_topic_name = f'rq{service_name}Request'
        res_topic_name = f'rr{service_name}Reply'

        # Create request writer (client sends requests)
        req_topic = Topic(self.domain_participant, req_topic_name, request_type, qos=qos)
        req_publisher = Publisher(self.domain_participant, qos=qos)
        req_writer = DataWriter(req_publisher, req_topic)

        # Create response reader (client receives responses)
        res_topic = Topic(self.domain_participant, res_topic_name, response_type, qos=qos)
        res_reader = DataReader(self.domain_participant, res_topic)

        # Store client
        self.clients[service_name] = {
            'request_writer': req_writer,
            'response_reader': res_reader,
            'request_type': request_type,
            'response_type': response_type,
            'sequence_number': 0
        }

        return self.clients[service_name]

    def dds_call_service(self, client, request, timeout: float = 5.0):
        """
        Call a service synchronously (blocking).

        :param client: The client dictionary returned by dds_create_client
        :param request: The request message
        :param timeout: Timeout in seconds
        :return: Response message or None if timeout

        Example:
            client = node.dds_create_client('/add_two_ints', ReqType_, ResType_)
            request = ReqType_(a=5, b=3)
            response = node.dds_call_service(client, request)
        """
        # Send request
        client['request_writer'].write(request)

        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            samples = client['response_reader'].take(N=1)
            if samples:
                return samples[0]
            time.sleep(0.01)

        print(f'Service call timeout after {timeout}s')
        return None

    def dds_spin(self):
        """
        Keep the node running (similar to rclpy.spin).

        This will block and keep all callbacks active.
        Press Ctrl+C to stop.

        Example:
            node = DDSNode(name='my_node')
            node.dds_create_subscription('/topic1', Type1_, callback1)
            node.dds_create_subscription('/topic2', Type2_, callback2)
            node.dds_spin()  # Blocks here, callbacks will be triggered automatically
        """
        self._running = True
        print(
            f'[{self.name}] Spinning with {len(self.subscribers)} '
            f'subscribers and {len(self.publishers)} publishers...')

        try:
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print(f'\n[{self.name}] Shutting down...')
            self._running = False

    def dds_spin_once(self, timeout: float = 0.1):
        """
        Spin once with a timeout (similar to rclpy.spin_once).

        :param timeout: Maximum time to wait in seconds

        Example:
            while True:
                node.dds_spin_once(timeout=0.1)
                # Do other work here
        """
        time.sleep(timeout)

    def dds_destroy_node(self):
        """
        Cleanup all resources (similar to Node.destroy_node).

        Example:
            node = DDSNode()
            # ... use node ...
            node.dds_destroy_node()
        """
        self._running = False
        self.subscribers.clear()
        self.publishers.clear()
        print(f'[{self.name}] Node destroyed.')
