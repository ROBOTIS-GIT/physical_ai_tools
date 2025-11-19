#!/usr/bin/env python3
"""SendCommand service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import TaskInfo as TaskInfo


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SendCommand_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SendCommand_Request_'):
    """SendCommand service Request."""

    ########################################
    # Constants
    ########################################
    IDLE = 0
    START_RECORD = 1
    START_INFERENCE = 2
    STOP = 3
    MOVE_TO_NEXT = 4
    RERECORD = 5
    FINISH = 6
    SKIP_TASK = 7

    command: int
    task_info: TaskInfo
