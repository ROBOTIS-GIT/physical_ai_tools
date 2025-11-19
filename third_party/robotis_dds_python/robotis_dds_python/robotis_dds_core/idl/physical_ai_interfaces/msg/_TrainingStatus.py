#!/usr/bin/env python3
"""TrainingStatus message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import TrainingInfo


@dataclass
@annotate.final
@annotate.autoid('sequential')
class TrainingStatus(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::TrainingStatus_'):
    """TrainingStatus message."""

    training_info: TrainingInfo
    current_step: int
    current_loss: float
    is_training: bool = False
    error: str = ''
