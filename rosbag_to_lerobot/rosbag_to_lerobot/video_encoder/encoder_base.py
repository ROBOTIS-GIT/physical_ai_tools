#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

from pathlib import Path
from typing import List, Optional, Union

import numpy as np


class VideoEncoder:

    def __init__(
        self,
        fps: int = 30,
        vcodec: str = 'libx264',
        pix_fmt: str = 'yuv420p',
        g: Optional[int] = 2,
        crf: Optional[int] = 23,
        qp: Optional[int] = None,
        fast_decode: int = 0,
    ):
        self.buffer = []
        self.fps = fps
        self.vcodec = vcodec
        self.pix_fmt = pix_fmt
        self.g = g
        self.crf = crf
        self.qp = qp
        self.fast_decode = fast_decode

    def set_buffer(self, frames: List[np.ndarray]) -> None:
        self.buffer = frames

    def clear_buffer(self) -> None:
        self.buffer = []

    def encode_video(self, video_path: Union[str, Path]) -> None:
        raise NotImplementedError('Must be implemented by subclasses')
