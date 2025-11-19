#!/usr/bin/env python3
"""TaskInfo message."""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate


@dataclass
@annotate.final
@annotate.autoid('sequential')
class TaskInfo(idl.IdlStruct, typename='physical_ai_interfaces::msg::dds_::TaskInfo_'):
    """TaskInfo message."""

    warmup_time_s: int
    episode_time_s: int
    reset_time_s: int
    num_episodes: int
    fps: int
    task_name: str = ''
    task_type: str = ''
    user_id: str = ''
    task_instruction: list[str] = field(default_factory=list)
    policy_path: str = ''
    tags: list[str] = field(default_factory=list)
    push_to_hub: bool = False
    private_mode: bool = False
    use_optimized_save_mode: bool = False
    record_inference_mode: bool = False
