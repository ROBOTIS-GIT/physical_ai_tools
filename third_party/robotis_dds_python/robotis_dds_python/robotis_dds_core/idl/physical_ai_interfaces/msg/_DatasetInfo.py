#!/usr/bin/env python3
"""DatasetInfo message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class DatasetInfo(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::DatasetInfo_'):
    """DatasetInfo message."""

    total_episodes: int
    total_tasks: int
    fps: int
    codebase_version: str = ''
    robot_type: str = ''
