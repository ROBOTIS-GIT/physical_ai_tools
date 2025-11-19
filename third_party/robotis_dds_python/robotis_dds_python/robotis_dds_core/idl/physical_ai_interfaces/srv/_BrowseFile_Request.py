#!/usr/bin/env python3
"""BrowseFile service Request message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class BrowseFile_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::BrowseFile_Request_'):
    """BrowseFile service Request."""

    action: str = ''
    current_path: str = ''
    target_name: str = ''
    target_files: list[str] = field(default_factory=list)
    target_folders: list[str] = field(default_factory=list)
