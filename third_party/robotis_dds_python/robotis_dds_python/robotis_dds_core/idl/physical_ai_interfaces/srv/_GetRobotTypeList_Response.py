#!/usr/bin/env python3
"""GetRobotTypeList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetRobotTypeList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetRobotTypeList_Response_'):
    """GetRobotTypeList service Response."""

    robot_types: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
