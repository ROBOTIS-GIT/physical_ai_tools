#!/usr/bin/env python3
"""GetUserList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetUserList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetUserList_Response_'):
    """GetUserList service Response."""

    user_list: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
