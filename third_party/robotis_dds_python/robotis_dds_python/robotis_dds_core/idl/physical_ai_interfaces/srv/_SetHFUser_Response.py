#!/usr/bin/env python3
"""SetHFUser service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class SetHFUser_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::SetHFUser_Response_'):
    """SetHFUser service Response."""

    user_id_list: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
