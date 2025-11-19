#!/usr/bin/env python3
"""GetRobotTypeList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetRobotTypeList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetRobotTypeList_Request_'):
    """GetRobotTypeList service Request."""

    pass
