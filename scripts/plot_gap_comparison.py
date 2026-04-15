#!/usr/bin/env python3
"""Compare per-topic segment-boundary lags between two merged episodes.

Intended for auditing merger improvements: run before and after the
per-topic gap-closure change, point this script at the two episode
directories, and see a bar chart of each topic's worst boundary lag
(max of pre_lag + post_lag across all segment boundaries).

Usage:
    python3 plot_gap_comparison.py <before_episode_dir> <after_episode_dir> \
        [--save out.png] [--show]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageOptions,
)


def _collect_times(episode_dir: Path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(episode_dir), storage_id='mcap'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    out: dict[str, list[int]] = {}
    while reader.has_next():
        topic, _data, t = reader.read_next()
        out.setdefault(topic, []).append(t)
    del reader
    for t in out.values():
        t.sort()
    return out


def _boundaries(episode_dir: Path, by_topic):
    info_path = episode_dir / 'episode_info.json'
    info = json.loads(info_path.read_text()) if info_path.exists() else {}
    segs = info.get('segments', []) or []
    if len(segs) < 2:
        return []
    all_ts = [t for ts in by_topic.values() for t in ts]
    if not all_ts:
        return []
    t0 = min(all_ts)
    t_end = max(all_ts)
    last_end_frame = segs[-1]['frame_duration'][1]
    scale = ((t_end - t0) / max(1, last_end_frame))
    return [t0 + int(seg['frame_duration'][1] * scale) for seg in segs[:-1]]


def _worst_gap_per_topic(episode_dir: Path):
    """Return { topic: worst (pre_lag + post_lag) in seconds }."""
    by_topic = _collect_times(episode_dir)
    bounds = _boundaries(episode_dir, by_topic)
    result: dict[str, float] = {}
    for topic, ts in by_topic.items():
        worst = 0.0
        for tb in bounds:
            pre = None
            post = None
            for t in ts:
                if t <= tb:
                    pre = t
                else:
                    post = t
                    break
            if pre is None or post is None:
                continue
            gap = (post - pre) / 1e9
            if gap > worst:
                worst = gap
        result[topic] = worst
    return result


def plot(before_dir: Path, after_dir: Path,
         out_path: Path = None, show: bool = False):
    before = _worst_gap_per_topic(before_dir)
    after = _worst_gap_per_topic(after_dir)

    topics = sorted(set(before) | set(after),
                    key=lambda t: -max(before.get(t, 0), after.get(t, 0)))
    before_vals = [before.get(t, 0) * 1000 for t in topics]   # ms
    after_vals = [after.get(t, 0) * 1000 for t in topics]

    y = list(range(len(topics)))
    fig, ax = plt.subplots(figsize=(11, max(4, 0.32 * len(topics))))
    h = 0.4
    ax.barh([v - h / 2 for v in y], before_vals, height=h,
            color='#d97706', label=f'before  ({before_dir.name})')
    ax.barh([v + h / 2 for v in y], after_vals, height=h,
            color='#2563eb', label=f'after   ({after_dir.name})')

    # Print percent improvement next to the after bar.
    for i, t in enumerate(topics):
        b = before.get(t, 0) * 1000
        a = after.get(t, 0) * 1000
        if b > 0:
            pct = (b - a) / b * 100
            ax.text(max(a, b) + 2, i + h / 2,
                    f'{pct:+.0f}%',
                    va='center', fontsize=7,
                    color='#2563eb' if pct >= 0 else '#b91c1c')

    ax.set_yticks(y)
    ax.set_yticklabels(topics, fontsize=7)
    ax.invert_yaxis()
    ax.set_xlabel('worst boundary gap across all segments [ms]')
    ax.set_title('Per-topic segment-boundary lag — before vs after',
                 fontsize=10)
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(True, axis='x', linestyle=':', alpha=0.5)
    fig.tight_layout()

    if out_path:
        fig.savefig(out_path, dpi=140)
        print(f'Saved: {out_path}')
    if show:
        plt.show()
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('before_dir', type=Path)
    ap.add_argument('after_dir', type=Path)
    ap.add_argument('--save', type=Path, default=None)
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args(argv)

    if not args.show and args.save is None:
        args.save = args.after_dir / 'gap_comparison.png'
    if not args.show:
        matplotlib.use('Agg')
    return plot(args.before_dir, args.after_dir, args.save, args.show)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
