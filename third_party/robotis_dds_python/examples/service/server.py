#!/usr/bin/env python3
"""
Service Server Example - SetRobotType.

Demonstrates how to create a DDS service server that can be called from:
- DDS clients (using this library)
- ROS 2 service clients (ros2 service call)

The service validates robot types and returns success/failure responses.

Usage:
    python server.py

Test with DDS client:
    python client.py ai_worker

Test with ROS 2:
    ros2 service call /set_robot_type
        physical_ai_interfaces/srv/SetRobotType
        "{robot_type: 'ai_worker'}"
"""

from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv import (
    SetRobotType_Request,
    SetRobotType_Response,
)
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    """Run SetRobotType service server example."""
    print('=== DDS Service Server Example ===')
    print('Service: /set_robot_type')
    print('Type: physical_ai_interfaces/srv/SetRobotType')
    print('Domain ID: 30')
    print()
    print('Valid robot types: ai_worker, omx, omy')
    print()
    print('Test with ROS 2:')
    print('  ros2 service call /set_robot_type \\')
    print('    physical_ai_interfaces/srv/SetRobotType \\')
    print('    "{robot_type: \'ai_worker\'}"')
    print()
    print('Press Ctrl+C to stop')
    print()

    # Create DDS node
    node = DDSNode(
        name='robot_type_server',
        domain_id=30,
        network_interface='auto',
        allow_multicast=True
    )

    # Service callback function
    def set_robot_type_callback(request):
        """
        Handle SetRobotType service requests.

        Args:
            request: SetRobotType_Request with robot_type field

        Returns:
            SetRobotType_Response with success and message fields
        """
        print(f'[Service Request] robot_type = "{request.robot_type}"')

        # Define valid ROBOTIS robot types
        valid_types = ['ai_worker', 'omx', 'omy']

        # Validate and respond
        if request.robot_type in valid_types:
            print(f'  ✓ Valid robot type: {request.robot_type}')
            return SetRobotType_Response(
                success=True,
                message=f'Robot type set to {request.robot_type}'
            )
        else:
            print(f'  ✗ Invalid robot type: {request.robot_type}')
            valid_types_str = ', '.join(valid_types)
            return SetRobotType_Response(
                success=False,
                message=(
                    f'Invalid robot type "{request.robot_type}". '
                    f'Valid types: {valid_types_str}'
                )
            )

    # Create service server
    node.dds_create_service(
        '/set_robot_type',
        SetRobotType_Request,
        SetRobotType_Response,
        set_robot_type_callback
    )

    print('Service server ready! Waiting for requests...')
    print()

    try:
        # Keep the service running
        node.dds_spin()
    except KeyboardInterrupt:
        print('\nStopping service server...')
    finally:
        node.dds_destroy_node()
        print('Service server stopped.')


if __name__ == '__main__':
    main()
