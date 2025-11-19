"""
Robotis DDS Python SDK
----------------------

High-level wrapper around CycloneDDS communication layer for ROS2-compatible
Robotis systems.

This package provides the `RobotisDDSSDK` class — a unified API to publish and
subscribe to core robot topics without direct ROS2 dependencies.

Example:
    >>> from robotis_dds_python.robotis_dds_sdk import RobotisDDSSDK
    >>> rds = RobotisDDSSDK(domain_id=30)
    >>> joints = rds.get_joint_state()
    >>> print(joints)
"""

from .robotis_dds_sdk import RobotisDDSSDK

__all__ = ["RobotisDDSSDK"]
