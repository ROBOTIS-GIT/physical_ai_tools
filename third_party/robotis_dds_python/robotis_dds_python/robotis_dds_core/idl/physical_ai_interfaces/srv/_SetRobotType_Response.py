#!/usr/bin/env python3
"""SetRobotType service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SetRobotType_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SetRobotType_Response_'):
    """SetRobotType service Response."""

    success: bool = False
    message: str = ''
