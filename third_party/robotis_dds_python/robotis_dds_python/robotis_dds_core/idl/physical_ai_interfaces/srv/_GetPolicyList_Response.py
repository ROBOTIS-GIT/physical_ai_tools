#!/usr/bin/env python3
"""GetPolicyList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetPolicyList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetPolicyList_Response_'):
    """GetPolicyList service Response."""

    policy_list: list[str] = field(default_factory=list)
    device_list: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
