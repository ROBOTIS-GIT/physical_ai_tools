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
    # Per-topic last emitted timestamp (output ns).
    last_out_t_per_topic: dict[str, int] = {}
    # Per-topic stitch points: output timestamp (ns) of each topic's first
    # message in segments after the first. Visualizers use these to draw
    # per-topic boundary annotations.
    stitch_times_per_topic: dict[str, list] = {}
    # Globally last emitted timestamp — fallback for new-topic offset.
    global_last_out_t = None
    # 1 ms epsilon to guarantee strict monotonicity at the boundary.
    EPSILON_NS = 1_000_000
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

            # Slurp all messages so we can inspect per-topic first times
            # before deciding on a single segment-wide offset.
            messages = []
            first_t_in_seg: dict[str, int] = {}
            while reader.has_next():
                topic, data, t = reader.read_next()
                messages.append((topic, data, t))
                first_t_in_seg.setdefault(topic, t)
            del reader

            # Single global offset: for each topic that existed in the
            # previous segment we need  C > last_X_out - first_X_orig  to
            # keep monotonicity. Taking the max across topics gives the
            # tightest common offset that satisfies every topic. Adding
            # EPSILON_NS gives strict '>'.
            #
            # Result: the "bottleneck" topic (fastest rate, published
            # closest to segment end) gets a gap ≈ EPSILON, while slower
            # topics get a gap ≈ their natural stride — because the
            # wall-clock phase relationship is preserved. All topics
            # transition at the same global timestamp, so the TF tree
            # stays coherent across boundaries.
            if seg_idx == 0 or not close_gaps:
                segment_offset = 0
            else:
                min_required = []
                for topic, first_t in first_t_in_seg.items():
                    if topic in last_out_t_per_topic:
                        min_required.append(
                            last_out_t_per_topic[topic] - first_t)
                if min_required:
                    segment_offset = max(min_required) + EPSILON_NS
                elif global_last_out_t is not None:
                    globally_first_t = min(first_t_in_seg.values())
                    segment_offset = (
                        global_last_out_t + frame_interval_ns
                        - globally_first_t)
                else:
                    segment_offset = 0

            stitched_first_seen: set = set()
            for topic, data, t in messages:
                new_t = t + segment_offset
                writer.write(topic, data, new_t)
                if seg_idx > 0 and topic not in stitched_first_seen:
                    stitch_times_per_topic.setdefault(
                        topic, []).append(new_t)
                    stitched_first_seen.add(topic)
                last_out_t_per_topic[topic] = new_t
                if global_last_out_t is None or new_t > global_last_out_t:
                    global_last_out_t = new_t
                total_messages += 1
    finally:
        del writer

    print(f'[mcap_merger] Merged {total_messages} messages from '
          f'{len(seg_list)} segment(s) -> {output_uri}')
    return Path(output_uri), stitch_times_per_topic


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
    out, _ = merge_segments_to(segs, episode_dir / 'merged')
    return out
