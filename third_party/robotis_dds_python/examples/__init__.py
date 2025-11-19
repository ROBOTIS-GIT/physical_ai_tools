"""
Examples Package.

DDS-ROS2 communication examples using robotis_dds_python.

Subpackages:
    topic: Basic topic publisher/subscriber examples
    service: Service server/client examples using physical_ai_interfaces
    advanced: Advanced integration examples

Quick Start:
    # Topic Publisher
    python -m examples.topic.publisher

    # Topic Subscriber
    python -m examples.topic.subscriber

    # Service Server
    python -m examples.service.server

    # Service Client
    python -m examples.service.client ai_worker
"""

from . import advanced, service, topic

__all__ = ['topic', 'service', 'advanced']
