#!/usr/bin/env python3
"""GetDatasetInfo service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetDatasetInfo_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetDatasetInfo_Request_'):
    """GetDatasetInfo service Request."""

    dataset_path: str = ''
