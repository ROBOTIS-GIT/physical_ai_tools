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

import json
import logging
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Union


# Public export list
__all__ = [
    'read_json',
    'write_json',
    'read_json_file',
    'read_jsonl',
    'write_jsonl',
    'safe_mkdir',
    'FileIO'
]

_logger = logging.getLogger('file_utils')
if not _logger.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
    _logger.addHandler(handler)
    _logger.setLevel(logging.INFO)


def safe_mkdir(path: Union[str, Path]) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


def read_json(
        file_path: Union[str, Path],
        *, default: Optional[Dict] = None,
        silent: bool = False) -> Optional[Dict]:
    p = Path(file_path)
    if p.exists() and not p.is_file():
        if not silent:
            _logger.warning(f'Path exists but not a file: {p}')
        return default
    try:
        text = p.read_text(encoding='utf-8')
        if text.strip() == '':
            if not silent:
                _logger.warning(f'Empty JSON file: {p}')
            return default
        return json.loads(text)
    except FileNotFoundError:
        if not silent:
            _logger.debug(f'JSON file not found: {p}')
        return default
    except json.JSONDecodeError as e:
        if not silent:
            _logger.error(f'JSON decode error in {p}: {e}')
        return default
    except Exception as e:  # noqa: BLE001
        if not silent:
            _logger.error(f'Unexpected error reading {p}: {e}')
        return default


def write_json(
    file_path: Union[str, Path], data: Dict, *, indent: int = 2
) -> bool:
    p = Path(file_path)
    try:
        safe_mkdir(p.parent)
        p.write_text(json.dumps(data, indent=indent), encoding='utf-8')
        return True
    except Exception as e:  # noqa: BLE001
        _logger.error(f'Failed to write JSON {p}: {e}')
        return False


def read_jsonl(path: Union[str, Path]) -> List[Dict]:
    p = Path(path)
    if not p.exists():
        return []
    out: List[Dict] = []
    for i, line in enumerate(p.read_text(encoding='utf-8').splitlines()):
        if not line.strip():
            continue
        try:
            obj = json.loads(line)
            if isinstance(obj, dict):
                out.append(obj)
            else:
                _logger.debug(f'Line {i} not a dict in {p}')
        except json.JSONDecodeError:
            _logger.warning(f'Malformed JSONL line {i} in {p}')
    return out


def write_jsonl(objs: Iterable[Dict], path: Union[str, Path]) -> bool:
    p = Path(path)
    try:
        safe_mkdir(p.parent)
        with p.open('w', encoding='utf-8') as f:
            for o in objs:
                f.write(json.dumps(o, separators=(',', ':')) + '\n')
        return True
    except Exception as e:  # noqa: BLE001
        _logger.error(f'Failed to write JSONL {p}: {e}')
        return False


# Backward compatibility wrapper
def read_json_file(file_path: str) -> Optional[Dict]:  # Legacy API
    return read_json(file_path)


class FileIO:
    read_json = staticmethod(read_json)
    write_json = staticmethod(write_json)
    read_jsonl = staticmethod(read_jsonl)
    write_jsonl = staticmethod(write_jsonl)
    safe_mkdir = staticmethod(safe_mkdir)
