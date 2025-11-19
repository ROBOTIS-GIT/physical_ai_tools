#!/usr/bin/env python3
"""TrainingInfo message."""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class TrainingInfo(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::TrainingInfo_'):
    """TrainingInfo message."""

    seed: int
    num_workers: int
    batch_size: int
    steps: int
    eval_freq: int
    log_freq: int
    save_freq: int
    dataset: str = ''
    policy_type: str = ''
    output_folder_name: str = ''
    policy_device: str = ''
