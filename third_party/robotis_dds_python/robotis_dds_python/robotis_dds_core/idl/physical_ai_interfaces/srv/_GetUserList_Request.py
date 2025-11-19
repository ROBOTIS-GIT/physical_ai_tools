#!/usr/bin/env python3
"""GetUserList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetUserList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetUserList_Request_'):
    """GetUserList service Request."""

    pass
