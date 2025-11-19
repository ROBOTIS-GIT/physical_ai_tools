#!/usr/bin/env python3
"""GetImageTopicList service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetImageTopicList_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetImageTopicList_Request_'):
    """GetImageTopicList service Request."""

    pass
