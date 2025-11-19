#!/usr/bin/env python3
"""GetSavedPolicyList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetSavedPolicyList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetSavedPolicyList_Request_'):
    """GetSavedPolicyList service Request."""

    pass
