#!/usr/bin/env python3
"""EditDataset service Request message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class EditDataset_Request(idl.IdlStruct, typename='physical_ai_interfaces::srv::dds_::EditDataset_Request_'):
    """EditDataset service Request."""

    ########################################
    # Constants
    ########################################
    MERGE = 0
    DELETE = 1

    mode: int
    merge_dataset_list: list[str] = field(default_factory=list)
    delete_dataset_path: str = ''
    output_path: str = ''
    delete_episode_num: list[int] = field(default_factory=list)
    upload_huggingface: bool = False
