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

"""Merge per-segment rosbag2 directories into a single rosbag2 episode.

Uses `rosbag2_py.SequentialReader` / `SequentialWriter` so the output
directory contains a standard `metadata.yaml` + `{basename}_0.mcap`,
identical in layout to a single-shot rosbag2 recording.
"""

import statistics
from pathlib import Path

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageOptions,
)


def _segment_dirs(pending_dir: Path):
    segments_root = pending_dir / 'segments'
    if not segments_root.is_dir():
        return []
    dirs = [
        d for d in segments_root.iterdir()
        if d.is_dir() and d.name.isdigit()
    ]
    return sorted(dirs, key=lambda d: int(d.name))


def merge_segments_to(
        segment_dirs,
        output_uri,
        close_gaps: bool = True,
        camera_fps: int = 15) -> Path:
    """Merge rosbag2 segment directories into a single rosbag2 at `output_uri`.

    `output_uri` must be a path that does NOT yet exist as a rosbag2
    directory — rosbag2_py will create it and populate `metadata.yaml` +
    `<basename>_0.mcap`.

    When `close_gaps` is True (default) the wall-clock dead time between
    segments (user setup time between Save and the next Record) is
    removed: each segment's messages are shifted so the first message of
    segment `i` lands exactly one camera frame (`1/camera_fps` s) after
    the last message of segment `i-1`. Without this, playback holds the
    last image frame for several seconds before the next segment starts.

    Args:
        segment_dirs: iterable of rosbag2 segment directories (each must
            contain its own metadata.yaml + .mcap).
        output_uri: target directory for the merged rosbag2.
        close_gaps: If True, remove inter-segment wall-clock gaps.
        camera_fps: Camera rate used to size the inter-segment spacing.

    Returns:
        Path to the output directory.
    """
    output_uri = str(output_uri)
    seg_list = [str(p) for p in segment_dirs]
    if not seg_list:
        raise FileNotFoundError('No segment directories to merge')
    frame_interval_ns = int(1e9 / max(camera_fps, 1))

    writer = SequentialWriter()
    try:
        writer.open(
            StorageOptions(uri=output_uri, storage_id='mcap'),
            ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            ),
        )
    except Exception as e:
        raise RuntimeError(
            f'SequentialWriter.open failed for {output_uri}: {e}') from e

    registered = set()
    total_messages = 0
    # Per-topic last emitted timestamp and running median stride (in ns).
    last_out_t_per_topic: dict[str, int] = {}
    stride_per_topic: dict[str, int] = {}
    # Globally last emitted timestamp — used as the fallback anchor for
    # topics that appear for the first time in a later segment.
    global_last_out_t = None
    try:
        for seg_idx, seg in enumerate(seg_list):
            reader = SequentialReader()
            try:
                reader.open(
                    StorageOptions(uri=seg, storage_id='mcap'),
                    ConverterOptions(
                        input_serialization_format='cdr',
                        output_serialization_format='cdr',
                    ),
                )
            except Exception as e:
                raise RuntimeError(
                    f'SequentialReader.open failed for {seg}: {e}') from e
            for tm in reader.get_all_topics_and_types():
                if tm.name in registered:
                    continue
                writer.create_topic(tm)
                registered.add(tm.name)

            # Slurp this segment so we can compute per-topic first-time /
            # strides before writing.
            messages = []
            first_t_in_seg: dict[str, int] = {}
            prev_t_by_topic: dict[str, int] = {}
            diffs_by_topic: dict[str, list] = {}
            while reader.has_next():
                topic, data, t = reader.read_next()
                messages.append((topic, data, t))
                first_t_in_seg.setdefault(topic, t)
                if topic in prev_t_by_topic:
                    dt = t - prev_t_by_topic[topic]
                    if dt > 0:
                        diffs_by_topic.setdefault(topic, []).append(dt)
                prev_t_by_topic[topic] = t
            del reader

            # Update running stride estimate per topic (min of any segment's
            # median — favors tighter natural cadence).
            for topic, diffs in diffs_by_topic.items():
                if len(diffs) < 5:
                    continue
                med = int(statistics.median(diffs))
                if med <= 0:
                    continue
                existing = stride_per_topic.get(topic)
                stride_per_topic[topic] = med if existing is None \
                    else min(existing, med)

            # Segment-level offset (fallback for topics with no prior output).
            if seg_idx == 0 or not close_gaps or global_last_out_t is None:
                segment_offset = 0
            else:
                globally_first_t = min(first_t_in_seg.values())
                segment_offset = (global_last_out_t + frame_interval_ns) \
                    - globally_first_t

            # Per-topic offset: when a topic has been seen before, its first
            # message in this segment should land exactly `stride` after its
            # own last emitted timestamp. New topics fall back to
            # segment_offset so they don't drift arbitrarily.
            offsets_this_seg: dict[str, int] = {}
            for topic, first_t in first_t_in_seg.items():
                if not close_gaps or topic not in last_out_t_per_topic:
                    offsets_this_seg[topic] = segment_offset
                    continue
                stride = stride_per_topic.get(topic, frame_interval_ns)
                offsets_this_seg[topic] = (
                    last_out_t_per_topic[topic] + stride - first_t)

            for topic, data, t in messages:
                new_t = t + offsets_this_seg[topic]
                writer.write(topic, data, new_t)
                last_out_t_per_topic[topic] = new_t
                if global_last_out_t is None or new_t > global_last_out_t:
                    global_last_out_t = new_t
                total_messages += 1
    finally:
        del writer

    print(f'[mcap_merger] Merged {total_messages} messages from '
          f'{len(seg_list)} segment(s) -> {output_uri}')
    return Path(output_uri)


def merge_episode(episode_dir) -> Path:
    """Backward-compat wrapper: merge `{episode_dir}/segments/*/` in place.

    Retained so callers that still pass a single episode_dir keep working.
    Produces `{episode_dir}/merged.mcap` subdir with metadata.yaml + mcap.
    """
    episode_dir = Path(episode_dir)
    segs = _segment_dirs(episode_dir)
    if not segs:
        raise FileNotFoundError(
            f'No segments under {episode_dir}/segments')
    return merge_segments_to(segs, episode_dir / 'merged')
