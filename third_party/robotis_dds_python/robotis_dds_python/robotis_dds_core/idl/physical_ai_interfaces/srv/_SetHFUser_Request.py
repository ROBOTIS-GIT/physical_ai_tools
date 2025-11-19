#!/usr/bin/env python3
"""SetHFUser service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SetHFUser_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SetHFUser_Request_'):
    """SetHFUser service Request."""

    token: str = ''
