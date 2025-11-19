#!/usr/bin/env python3
"""ControlHfServer service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class ControlHfServer_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::ControlHfServer_Request_'):
    """ControlHfServer service Request."""

    mode: str = ''
    repo_id: str = ''
    local_dir: str = ''
    repo_type: str = ''
    author: str = ''
