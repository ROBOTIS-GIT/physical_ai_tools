#!/usr/bin/env python3
"""EditDataset service Response message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class EditDataset_Response(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::EditDataset_Response_'):
    """EditDataset service Response."""

    success: bool = False
    message: str = ''
