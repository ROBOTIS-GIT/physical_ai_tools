#!/usr/bin/env python3
"""TaskStatus message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import TaskInfo


@dataclass
@annotate.final
@annotate.autoid('sequential')
class TaskStatus(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::TaskStatus_'):
    """TaskStatus message."""

    ########################################
    # Constants
    ########################################
    READY = 0
    WARMING_UP = 1
    RESETTING = 2
    RECORDING = 3
    SAVING = 4
    STOPPED = 5
    INFERENCING = 6

    task_info: TaskInfo
    phase: int
    total_time: int
    proceed_time: int
    current_episode_number: int
    current_scenario_number: int
    encoding_progress: float
    used_storage_size: float
    total_storage_size: float
    used_cpu: float
    used_ram_size: float
    total_ram_size: float
    robot_type: str = ''
    current_task_instruction: str = ''
    error: str = ''
