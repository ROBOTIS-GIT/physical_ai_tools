#!/usr/bin/env python3
"""Inference service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class Inference_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::Inference_Response_'):
    """Inference service Response."""

    success: bool = False
    message: str = ''
    payload: bytes = b''
