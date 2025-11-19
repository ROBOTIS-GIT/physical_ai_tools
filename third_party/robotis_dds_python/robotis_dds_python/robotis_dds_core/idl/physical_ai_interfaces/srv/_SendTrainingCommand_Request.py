#!/usr/bin/env python3
"""SendTrainingCommand service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import TrainingInfo as TrainingInfo


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SendTrainingCommand_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SendTrainingCommand_Request_'):
    """SendTrainingCommand service Request."""

    ########################################
    # Constants
    ########################################
    NONE = 0
    START = 1
    FINISH = 2

    command: int
    training_info: TrainingInfo
    resume: bool = False
    resume_model_path: str = ''
