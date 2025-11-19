#!/usr/bin/env python3
"""SendTrainingCommand service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SendTrainingCommand_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SendTrainingCommand_Response_'):
    """SendTrainingCommand service Response."""

    success: bool = False
    message: str = ''
