#!/usr/bin/env python3
"""SendCommand service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SendCommand_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SendCommand_Response_'):
    """SendCommand service Response."""

    success: bool = False
    message: str = ''
