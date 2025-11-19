#!/usr/bin/env python3
"""GetModelWeightList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetModelWeightList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetModelWeightList_Response_'):
    """GetModelWeightList service Response."""

    model_weight_list: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
