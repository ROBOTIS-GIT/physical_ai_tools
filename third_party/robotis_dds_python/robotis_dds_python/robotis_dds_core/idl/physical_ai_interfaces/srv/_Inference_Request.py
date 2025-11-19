#!/usr/bin/env python3
"""Inference service Request message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class Inference_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::Inference_Request_'):
    """Inference service Request."""

    command: str = ''
    task_id: str = ''
    payload: bytes = b''
