#!/usr/bin/env python3
"""Zoom into each segment boundary and plot every topic's samples.

For each segment boundary this produces a single figure with one row
per topic, scatter-dotting each message's timestamp and first scalar
value within `±window` seconds of the boundary. A dashed vertical line
marks the boundary itself, so per-topic "dragging" at the stitch shows
up as widely-spaced dots straddling the line.

Usage:
    python3 plot_boundary_zoom.py <episode_dir> \
        [--window 0.5] [--out-dir DIR] [--show]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message


EVENT_ONLY_TYPES = {
    'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/Image',
    'tf2_msgs/msg/TFMessage',
}


def _extract_scalar(type_str, msg):
    """One representative float per message (for y-axis). Falls back to 0."""
    try:
        if type_str == 'sensor_msgs/msg/JointState':
            return float(msg.position[0]) if msg.position else 0.0
        if type_str == 'geometry_msgs/msg/PoseStamped':
            return float(msg.pose.position.x)
        if type_str == 'geometry_msgs/msg/Twist':
            return float(msg.linear.x)
        if type_str == 'nav_msgs/msg/Odometry':
            return float(msg.pose.pose.position.x)
        if type_str == 'sensor_msgs/msg/CameraInfo':
            return float(msg.width)
    except Exception:
        return 0.0
    return 0.0


def _collect(episode_dir: Path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(episode_dir), storage_id='mcap'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    type_map = {
        tm.name: tm.type for tm in reader.get_all_topics_and_types()
    }
    by_topic: dict[str, dict] = {
        t: {'type': type_map[t], 'times': [], 'values': []}
        for t in type_map
    }
    msg_cache: dict = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        entry = by_topic[topic]
        entry['times'].append(t)
        type_str = entry['type']
        if type_str in EVENT_ONLY_TYPES:
            entry['values'].append(None)
            continue
        cls = msg_cache.get(type_str)
        if cls is None:
            try:
                cls = get_message(type_str)
            except Exception:
                cls = False
            msg_cache[type_str] = cls
        if not cls:
            entry['values'].append(None)
            continue
        try:
            msg = deserialize_message(data, cls)
            entry['values'].append(_extract_scalar(type_str, msg))
        except Exception:
            entry['values'].append(None)
    del reader
    return by_topic


def _boundaries(episode_dir: Path, by_topic):
    """Return list of (boundary_index, fallback_t_ns, prev_primitive,
    per_topic_stitch_t_ns_dict).

    `per_topic_stitch_t_ns_dict[topic]` is the EXACT timestamp where
    that topic transitions into segment `boundary_index + 1`. When the
    merger wrote `stitch_times_ns` in episode_info.json we use those;
    otherwise we fall back to a linear estimate (less accurate, marked
    only by the global red line).
    """
    info = json.loads(
        (episode_dir / 'episode_info.json').read_text())
    segs = info.get('segments', []) or []
    if len(segs) < 2:
        return []
    all_ts = [t for e in by_topic.values() for t in e['times']]
    if not all_ts:
        return []
    t0 = min(all_ts)
    t_end = max(all_ts)
    last_end_frame = segs[-1]['frame_duration'][1]
    scale = (t_end - t0) / max(1, last_end_frame)
    stitch_times_ns = info.get('stitch_times_ns', {}) or {}
    out = []
    for i, seg in enumerate(segs[:-1]):
        fallback_t = t0 + int(seg['frame_duration'][1] * scale)
        per_topic = {
            topic: int(times[i])
            for topic, times in stitch_times_ns.items()
            if i < len(times)
        }
        out.append(
            (i, fallback_t, seg.get('primitive_description', ''), per_topic))
    return out


def plot_boundary(by_topic, b_idx, t_boundary, prev_prim, per_topic_stitch,
                  window_s, out_path, show):
    window_ns = int(window_s * 1e9)
    lo = t_boundary - window_ns
    hi = t_boundary + window_ns

    topics = sorted(by_topic)
    fig, axes = plt.subplots(
        len(topics), 1,
        figsize=(10, max(2, 0.5 * len(topics))),
        sharex=True, squeeze=False)
    axes = [ax[0] for ax in axes]

    has_anything = False
    for ax, topic in zip(axes, topics):
        entry = by_topic[topic]
        xs, ys = [], []
        for t, v in zip(entry['times'], entry['values']):
            if lo <= t <= hi:
                xs.append((t - t_boundary) / 1e9)
                ys.append(v if v is not None else 0)
        if xs:
            has_anything = True
            ax.scatter(xs, ys, s=10, alpha=0.8)

        # Per-topic boundary (preferred). Falls back to global if missing.
        topic_b = per_topic_stitch.get(topic)
        if topic_b is not None:
            ax.axvline((topic_b - t_boundary) / 1e9,
                       color='#16a34a', linestyle='--',
                       linewidth=0.9, alpha=0.85,
                       label='per-topic stitch')
        ax.axvline(0, color='red', linestyle=':', linewidth=0.6, alpha=0.5,
                   label='estimated global')
        ax.set_ylabel(topic.split('/')[-1], fontsize=7)
        ax.tick_params(axis='both', labelsize=6)
        ax.grid(True, linestyle=':', alpha=0.4)
        ax.set_title(topic, fontsize=7, loc='left')

    if not has_anything:
        print(f'  boundary {b_idx}: no samples in window, skipped')
        plt.close(fig)
        return

    axes[-1].set_xlabel(f'time relative to boundary [s]  '
                        f'(window ±{window_s}s)')
    fig.suptitle(
        f'Segment boundary {b_idx}  (after "{prev_prim}")',
        fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.98])

    if out_path:
        fig.savefig(out_path, dpi=140)
        print(f'  boundary {b_idx}: saved {out_path}')
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('episode_dir', type=Path)
    ap.add_argument('--window', type=float, default=0.5,
                    help='Half-window size in seconds (default: 0.5).')
    ap.add_argument('--out-dir', type=Path, default=None,
                    help='Output directory (default: '
                         '<episode_dir>/boundary_zoom).')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args(argv)

    out_dir = args.out_dir or (args.episode_dir / 'boundary_zoom')
    if not args.show:
        matplotlib.use('Agg')
        out_dir.mkdir(parents=True, exist_ok=True)

    by_topic = _collect(args.episode_dir)
    boundaries = _boundaries(args.episode_dir, by_topic)
    if not boundaries:
        print('No segment boundaries found.')
        return 0

    print(f'Generating {len(boundaries)} boundary zoom plot(s) '
          f'(window ±{args.window}s)')
    for b_idx, t_b, prev_prim, per_topic in boundaries:
        out = None if args.show else (
            out_dir / f'boundary_{b_idx:02d}.png')
        plot_boundary(by_topic, b_idx, t_b, prev_prim, per_topic,
                      args.window, out, args.show)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
