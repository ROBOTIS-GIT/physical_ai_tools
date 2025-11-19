#!/usr/bin/env python3
"""GetHFUser service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetHFUser_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetHFUser_Request_'):
    """GetHFUser service Request."""

    pass
