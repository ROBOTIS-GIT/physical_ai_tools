#!/usr/bin/env python3
"""HFOperationStatus message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class HFOperationStatus(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::HFOperationStatus_'):
    """HFOperationStatus message."""

    progress_current: int
    progress_total: int
    progress_percentage: float
    operation: str = ''
    status: str = ''
    local_path: str = ''
    repo_id: str = ''
    message: str = ''
