#!/usr/bin/env python3
"""GetTrainingInfo service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import TrainingInfo as TrainingInfo


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetTrainingInfo_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetTrainingInfo_Response_'):
    """GetTrainingInfo service Response."""

    training_info: TrainingInfo
    success: bool = False
    message: str = ''
