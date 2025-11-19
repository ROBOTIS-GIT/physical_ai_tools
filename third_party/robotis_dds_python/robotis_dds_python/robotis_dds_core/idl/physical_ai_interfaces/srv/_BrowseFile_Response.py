#!/usr/bin/env python3
"""BrowseFile service Response message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
from robotis_dds_python.robotis_dds_core.idl.physical_ai_interfaces.msg import BrowserItem


@dataclass
@annotate.final
@annotate.autoid('sequential')
class BrowseFile_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::BrowseFile_Response_'):
    """BrowseFile service Response."""

    success: bool = False
    message: str = ''
    current_path: str = ''
    parent_path: str = ''
    selected_path: str = ''
    items: list[BrowserItem] = field(default_factory=list)
