#!/usr/bin/env python3
"""BrowserItem message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class BrowserItem(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::BrowserItem_'):
    """BrowserItem message."""

    size: int
    name: str = ''
    full_path: str = ''
    is_directory: bool = False
    modified_time: str = ''
    has_target_file: bool = False
    