#!/usr/bin/env python3
"""
Service Client Example - SetRobotType.

Demonstrates how to call a DDS service from a client.
Sends requests to set robot type and receives responses.

Usage:
    python client.py [robot_type]

Examples:
    python client.py ai_worker
    python client.py omx
    python client.py omy
"""

import sys
import time

from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.srv import (
    SetRobotType_Request,
    SetRobotType_Response,
)
from robotis_dds_python.robotis_dds_core.tools.dds_node import DDSNode


def main():
    """Run SetRobotType service client example."""
    # Get robot type from command line or use default
    if len(sys.argv) > 1:
        robot_type = sys.argv[1]
    else:
        robot_type = 'ai_worker'

    print('=== DDS Service Client Example ===')
    print(f'Calling /set_robot_type service with robot_type="{robot_type}"')
    print('Domain ID: 30')
    print()

    # Create DDS node
    node = DDSNode(
        name='robot_type_client',
        domain_id=30,
        network_interface='auto',
        allow_multicast=True
    )

    # Create service client
    client = node.dds_create_client(
        '/set_robot_type',
        SetRobotType_Request,
        SetRobotType_Response
    )

    print('Waiting for service to become available...')
    time.sleep(1.0)  # Give time for service discovery

    try:
        # Create request
        request = SetRobotType_Request(robot_type=robot_type)
        print(f'\nSending request: robot_type = "{request.robot_type}"')

        # Call service
        response = client.call(request, timeout=5.0)

        if response:
            print('\n=== Response ===')
            print(f'Success: {response.success}')
            print(f'Message: {response.message}')

            if response.success:
                print('\n✓ Service call successful!')
            else:
                print('\n✗ Service call failed')
        else:
            print('\n✗ Service call timed out or failed')

    except KeyboardInterrupt:
        print('\nStopping client...')
    finally:
        node.dds_destroy_node()
        print('Client stopped.')


if __name__ == '__main__':
    main()
