#!/usr/bin/env python3
"""GetTrainingInfo service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetTrainingInfo_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetTrainingInfo_Request_'):
    """GetTrainingInfo service Request."""

    train_config_path: str = ''
