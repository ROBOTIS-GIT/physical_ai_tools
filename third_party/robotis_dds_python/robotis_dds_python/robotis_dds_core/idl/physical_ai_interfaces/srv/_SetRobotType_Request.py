#!/usr/bin/env python3
"""SetRobotType service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SetRobotType_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SetRobotType_Request_'):
    """SetRobotType service Request."""

    robot_type: str = ''
