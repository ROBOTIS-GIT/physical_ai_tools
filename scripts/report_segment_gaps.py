#!/usr/bin/env python3
"""Quantify per-topic timing gaps at segment boundaries in a merged episode.

For every segment boundary (positions taken from episode_info.json's
`frame_duration`), this script reports, per topic:

    * the boundary time t_b (seconds from t0)
    * t_pre  = time of the last message whose timestamp ≤ t_b
    * t_post = time of the first message whose timestamp > t_b
    * pre_lag  = t_b - t_pre   (how long before the boundary the topic
                                stopped publishing in this segment)
    * post_lag = t_post - t_b  (how long after the boundary the topic
                                started publishing in the next segment)
    * stride   = median inter-arrival interval of the topic (reference)

A healthy gap-closed merge has pre_lag / post_lag both ≈ stride. Much
larger values indicate that topic was effectively "frozen" across the
stitch.

Usage:
    python3 report_segment_gaps.py <episode_dir>
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageOptions,
)


def _collect(episode_dir: Path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(episode_dir), storage_id='mcap'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    by_topic: dict[str, list[int]] = {}
    while reader.has_next():
        topic, _data, t = reader.read_next()
        by_topic.setdefault(topic, []).append(t)
    del reader
    for t in by_topic.values():
        t.sort()
    return by_topic


def _median_stride(ts):
    if len(ts) < 2:
        return None
    diffs = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    return statistics.median(diffs) if diffs else None


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('episode_dir', type=Path)
    ap.add_argument('--top', type=int, default=0,
                    help='Show only the N topics with the largest worst-case '
                         'boundary lag. 0 = all topics.')
    args = ap.parse_args(argv)

    info = json.loads(
        (args.episode_dir / 'episode_info.json').read_text())
    segs = info.get('segments', []) or []
    fps = float(info.get('fps', 15))
    if len(segs) < 2:
        print('Only one segment — no boundaries to report.')
        return 0

    by_topic = _collect(args.episode_dir)
    if not by_topic:
        print('No messages found.')
        return 2

    t0 = min(min(ts) for ts in by_topic.values() if ts)
    t_end = max(max(ts) for ts in by_topic.values() if ts)
    duration_s = (t_end - t0) / 1e9

    last_end_frame = segs[-1]['frame_duration'][1]
    scale_s_per_frame = duration_s / max(1, last_end_frame)

    # Boundaries in absolute nanoseconds.
    boundaries = []
    for seg in segs[:-1]:
        end_frame = seg['frame_duration'][1]
        boundaries.append(t0 + int(end_frame * scale_s_per_frame * 1e9))

    worst_per_topic = {}
    rows = []  # flat list of dicts for printing

    for topic in sorted(by_topic):
        ts = by_topic[topic]
        stride_ns = _median_stride(ts)
        for b_i, tb in enumerate(boundaries):
            # Last message ≤ tb.
            pre = None
            post = None
            # linear scan is fine here (small N usually)
            for t in ts:
                if t <= tb:
                    pre = t
                else:
                    post = t
                    break
            pre_lag = (tb - pre) / 1e9 if pre is not None else None
            post_lag = (post - tb) / 1e9 if post is not None else None
            worst = max(filter(None, [pre_lag, post_lag]), default=0)
            worst_per_topic[topic] = max(
                worst_per_topic.get(topic, 0), worst)
            rows.append({
                'topic': topic,
                'boundary': b_i,
                'pre_lag_ms': pre_lag * 1000 if pre_lag is not None else None,
                'post_lag_ms': post_lag * 1000 if post_lag is not None else None,
                'stride_ms': stride_ns / 1e6 if stride_ns else None,
            })

    # Optionally filter to top-N offenders.
    if args.top > 0:
        top_topics = sorted(worst_per_topic,
                            key=worst_per_topic.get,
                            reverse=True)[:args.top]
        top_set = set(top_topics)
        rows = [r for r in rows if r['topic'] in top_set]

    # Print.
    hdr = (f'{"topic":60s}  {"seg":>3s}  '
           f'{"pre_lag_ms":>10s}  {"post_lag_ms":>11s}  {"stride_ms":>9s}')
    print(hdr)
    print('-' * len(hdr))
    for r in rows:
        pre = 'n/a' if r['pre_lag_ms'] is None else f"{r['pre_lag_ms']:.1f}"
        post = 'n/a' if r['post_lag_ms'] is None else f"{r['post_lag_ms']:.1f}"
        stride = 'n/a' if r['stride_ms'] is None else f"{r['stride_ms']:.1f}"
        print(f"{r['topic']:60s}  {r['boundary']:>3d}  "
              f'{pre:>10s}  {post:>11s}  {stride:>9s}')

    # Summary: topics sorted by worst boundary lag.
    print()
    print('Top offenders (worst boundary lag across all segments):')
    for t, v in sorted(worst_per_topic.items(),
                       key=lambda kv: kv[1], reverse=True)[:10]:
        print(f'  {v * 1000:>8.1f} ms   {t}')

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
