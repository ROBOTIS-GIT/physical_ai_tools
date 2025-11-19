#!/usr/bin/env python3
"""ControlHfServer service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class ControlHfServer_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::ControlHfServer_Response_'):
    """ControlHfServer service Response."""

    success: bool = False
    message: str = ''
