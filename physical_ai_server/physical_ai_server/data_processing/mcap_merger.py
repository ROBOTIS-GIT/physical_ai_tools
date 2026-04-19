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

"""Archive per-segment rosbag2 directories into a single multi-file episode.

Instead of merging messages into one mcap (which requires timestamp
shifting and introduces phase-spread artifacts), this module copies each
segment's mcap file into the episode directory as ``{ep}_{seg}.mcap``
and generates a unified ``metadata.yaml`` that references all files.
rosbag2 natively supports multi-file bags, so ``ros2 bag play/info``
work out of the box with no timestamp modification.
"""

import os
import shutil
from pathlib import Path

import yaml


def _segment_dirs(pending_dir: Path):
    segments_root = pending_dir / 'segments'
    if not segments_root.is_dir():
        return []
    dirs = [
        d for d in segments_root.iterdir()
        if d.is_dir() and d.name.isdigit()
    ]
    return sorted(dirs, key=lambda d: int(d.name))


def _parse_segment_metadata(seg_dir: Path):
    """Read a segment's metadata.yaml and return the parsed dict."""
    meta_path = seg_dir / 'metadata.yaml'
    if not meta_path.exists():
        raise FileNotFoundError(f'No metadata.yaml in {seg_dir}')
    with open(meta_path, 'r') as f:
        return yaml.safe_load(f)


def _find_mcap_file(seg_dir: Path):
    """Return the first .mcap file in seg_dir."""
    mcaps = sorted(seg_dir.glob('*.mcap'))
    if not mcaps:
        raise FileNotFoundError(f'No .mcap file in {seg_dir}')
    return mcaps[0]


def archive_segments(segment_dirs, output_dir, ep_idx=0):
    """Copy segment mcap files into `output_dir` and write a unified
    ``metadata.yaml``.

    Each segment's mcap is renamed to ``{ep_idx}_{seg_idx}.mcap``.
    Original timestamps are preserved byte-for-byte — no shifting,
    quantization, or rewriting.

    Args:
        segment_dirs: ordered iterable of rosbag2 segment directories.
        output_dir: target directory (must already exist).
        ep_idx: episode index used in file naming.

    Returns:
        list of output mcap file paths.
    """
    seg_list = [Path(p) for p in segment_dirs]
    if not seg_list:
        raise FileNotFoundError('No segment directories to archive')

    os.makedirs(output_dir, exist_ok=True)

    output_files = []
    file_entries = []
    all_topic_counts = {}  # topic_name -> {meta, count}
    global_start = None
    global_end = None
    total_messages = 0
    ros_distro = 'jazzy'

    for seg_idx, seg_dir in enumerate(seg_list):
        # Copy mcap file
        src_mcap = _find_mcap_file(seg_dir)
        dst_name = f'{ep_idx}_{seg_idx}.mcap'
        dst_path = Path(output_dir) / dst_name
        shutil.copy2(str(src_mcap), str(dst_path))
        output_files.append(dst_path)

        # Parse segment metadata
        meta = _parse_segment_metadata(seg_dir)
        bag_info = meta.get('rosbag2_bagfile_information', {})

        start_ns = bag_info.get('starting_time', {}).get(
            'nanoseconds_since_epoch', 0)
        dur_ns = bag_info.get('duration', {}).get('nanoseconds', 0)
        msg_count = bag_info.get('message_count', 0)
        ros_distro = bag_info.get('ros_distro', ros_distro)

        end_ns = start_ns + dur_ns
        if global_start is None or start_ns < global_start:
            global_start = start_ns
        if global_end is None or end_ns > global_end:
            global_end = end_ns
        total_messages += msg_count

        file_entries.append({
            'path': dst_name,
            'starting_time': {
                'nanoseconds_since_epoch': start_ns,
            },
            'duration': {
                'nanoseconds': dur_ns,
            },
            'message_count': msg_count,
        })

        # Accumulate per-topic message counts
        for entry in bag_info.get('topics_with_message_count', []):
            tm = entry.get('topic_metadata', {})
            name = tm.get('name', '')
            count = entry.get('message_count', 0)
            if name in all_topic_counts:
                all_topic_counts[name]['count'] += count
            else:
                all_topic_counts[name] = {
                    'meta': tm,
                    'count': count,
                }

    # Build unified metadata.yaml
    topics_with_count = []
    for name in sorted(all_topic_counts):
        entry = all_topic_counts[name]
        topics_with_count.append({
            'topic_metadata': entry['meta'],
            'message_count': entry['count'],
        })

    unified = {
        'rosbag2_bagfile_information': {
            'version': 9,
            'storage_identifier': 'mcap',
            'duration': {
                'nanoseconds': (global_end - global_start)
                if global_start and global_end else 0,
            },
            'starting_time': {
                'nanoseconds_since_epoch': global_start or 0,
            },
            'message_count': total_messages,
            'topics_with_message_count': topics_with_count,
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [f['path'] for f in file_entries],
            'files': file_entries,
            'custom_data': None,
            'ros_distro': ros_distro,
        }
    }

    meta_path = Path(output_dir) / 'metadata.yaml'
    with open(meta_path, 'w') as f:
        yaml.dump(unified, f, default_flow_style=False, sort_keys=False)

    print(f'[mcap_merger] Archived {len(seg_list)} segment(s) '
          f'({total_messages} messages) -> {output_dir}')
    return output_files
