#!/usr/bin/env python3
"""GetDatasetList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetDatasetList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetDatasetList_Response_'):
    """GetDatasetList service Response."""

    dataset_list: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
