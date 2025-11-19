#!/usr/bin/env python3
"""GetSavedPolicyList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetSavedPolicyList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetSavedPolicyList_Response_'):
    """GetSavedPolicyList service Response."""

    saved_policy_path: list[str] = field(default_factory=list)
    saved_policy_type: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
