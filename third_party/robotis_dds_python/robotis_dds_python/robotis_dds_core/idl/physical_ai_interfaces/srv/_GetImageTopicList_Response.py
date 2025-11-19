#!/usr/bin/env python3
"""GetImageTopicList service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class GetImageTopicList_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::GetImageTopicList_Response_'):
    """GetImageTopicList service Response."""

    image_topic_list: list[str] = field(default_factory=list)
    success: bool = False
    message: str = ''
