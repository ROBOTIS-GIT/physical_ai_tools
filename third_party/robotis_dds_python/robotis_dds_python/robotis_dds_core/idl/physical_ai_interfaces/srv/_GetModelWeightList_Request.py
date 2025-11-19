#!/usr/bin/env python3
"""GetModelWeightList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetModelWeightList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetModelWeightList_Request_'):
    """GetModelWeightList service Request."""

    pass
