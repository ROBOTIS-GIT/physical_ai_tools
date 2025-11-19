#!/usr/bin/env python3
"""GetPolicyList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetPolicyList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetPolicyList_Request_'):
    """GetPolicyList service Request."""

    pass
