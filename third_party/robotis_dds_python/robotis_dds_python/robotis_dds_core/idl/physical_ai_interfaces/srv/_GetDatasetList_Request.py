#!/usr/bin/env python3
"""GetDatasetList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetDatasetList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetDatasetList_Request_'):
    """GetDatasetList service Request."""

    user_id: str = ''
