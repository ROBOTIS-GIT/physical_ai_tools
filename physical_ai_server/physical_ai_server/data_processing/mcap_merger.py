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

"""Merge per-segment rosbag2 .mcap files into a single episode .mcap.

Layout:
    {episode_dir}/segments/{segment_idx}/<rosbag>.mcap
    {episode_dir}/merged.mcap   <- output
"""

from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer


def _segment_dirs(episode_dir: Path):
    segments_root = episode_dir / 'segments'
    if not segments_root.is_dir():
        return []
    dirs = [
        d for d in segments_root.iterdir()
        if d.is_dir() and d.name.isdigit()
    ]
    return sorted(dirs, key=lambda d: int(d.name))


def merge_episode(episode_dir) -> Path:
    """Merge all segments under `episode_dir` into `episode_dir/merged.mcap`.

    Segments are processed in numeric order; messages within each segment
    are appended in their stored order. Schemas/channels are de-duplicated
    across segments.

    Returns:
        Path to the produced `merged.mcap`.

    Raises:
        FileNotFoundError if no segments / no .mcap inputs.
    """
    episode_dir = Path(episode_dir)
    seg_dirs = _segment_dirs(episode_dir)
    if not seg_dirs:
        raise FileNotFoundError(
            f'No segments under {episode_dir}/segments')

    output_path = episode_dir / 'merged.mcap'
    tmp_path = episode_dir / '.merged.mcap.tmp'

    schema_id_map = {}   # (name, encoding, data_bytes) -> new schema id
    channel_id_map = {}  # (topic, schema_id, encoding) -> new channel id
    total_messages = 0
    used_segments = 0

    with open(tmp_path, 'wb') as out_stream:
        writer = Writer(out_stream)
        writer.start(profile='ros2', library='physical_ai_server')

        for seg_dir in seg_dirs:
            mcap_files = sorted(seg_dir.glob('*.mcap'))
            if not mcap_files:
                continue
            used_segments += 1
            for mcap_path in mcap_files:
                with open(mcap_path, 'rb') as in_stream:
                    reader = make_reader(in_stream)
                    for schema, channel, message in reader.iter_messages():
                        if schema is None:
                            new_schema_id = 0
                        else:
                            schema_key = (
                                schema.name,
                                schema.encoding,
                                bytes(schema.data),
                            )
                            if schema_key not in schema_id_map:
                                schema_id_map[schema_key] = \
                                    writer.register_schema(
                                        name=schema.name,
                                        encoding=schema.encoding,
                                        data=schema.data,
                                    )
                            new_schema_id = schema_id_map[schema_key]

                        channel_key = (
                            channel.topic,
                            new_schema_id,
                            channel.message_encoding,
                        )
                        if channel_key not in channel_id_map:
                            channel_id_map[channel_key] = \
                                writer.register_channel(
                                    topic=channel.topic,
                                    message_encoding=channel.message_encoding,
                                    schema_id=new_schema_id,
                                    metadata=dict(channel.metadata or {}),
                                )
                        new_channel_id = channel_id_map[channel_key]

                        writer.add_message(
                            channel_id=new_channel_id,
                            log_time=message.log_time,
                            data=message.data,
                            publish_time=message.publish_time,
                            sequence=message.sequence,
                        )
                        total_messages += 1

        writer.finish()

    if used_segments == 0:
        try:
            tmp_path.unlink()
        except OSError:
            pass
        raise FileNotFoundError(
            f'No .mcap files found in any segment of {episode_dir}')

    tmp_path.replace(output_path)
    print(f'[mcap_merger] Merged {total_messages} messages from '
          f'{used_segments} segment(s) -> {output_path}')
    return output_path
