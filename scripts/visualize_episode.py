#!/usr/bin/env python3
"""Plot the time-series of every non-image topic in a merged episode.

Useful for eyeballing the continuity of a bag that was stitched together
from multiple scratch segments. Any topic whose type can be deserialized
to scalar fields is drawn; image/compressed-image topics are shown as a
per-topic event plot instead of attempted value plots.

Segment boundaries (from episode_info.json's `frame_duration`) are drawn
as dashed vertical lines so you can tell whether joints / pose / odom /
etc. flow smoothly across the cuts.

Usage:
    python3 visualize_episode.py <episode_dir> [--save out.png] [--show]
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


# Topics whose type we don't try to plot values for; events-only.
EVENT_ONLY_TYPES = {
    'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/Image',
    'tf2_msgs/msg/TFMessage',
}


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
    by_topic = {
        name: {'type': type_map[name], 'times': [], 'values': []}
        for name in type_map
    }
    msg_cache = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        entry = by_topic[topic]
        entry['times'].append(t)
        type_str = entry['type']
        if type_str in EVENT_ONLY_TYPES:
            continue
        cls = msg_cache.get(type_str)
        if cls is None:
            try:
                cls = get_message(type_str)
            except Exception:
                cls = False  # mark as "won't try again"
            msg_cache[type_str] = cls
        if not cls:
            continue
        try:
            msg = deserialize_message(data, cls)
        except Exception:
            continue
        vec = _extract_scalars(type_str, msg)
        if vec is not None:
            entry['values'].append(vec)
    del reader
    return by_topic


def _extract_scalars(type_str, msg):
    """Return a list of floats (one "frame") for known message types."""
    if type_str == 'sensor_msgs/msg/JointState':
        return list(msg.position)
    if type_str == 'geometry_msgs/msg/PoseStamped':
        p = msg.pose.position
        q = msg.pose.orientation
        return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if type_str == 'geometry_msgs/msg/Twist':
        return [msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z]
    if type_str == 'nav_msgs/msg/Odometry':
        p = msg.pose.pose.position
        tw = msg.twist.twist
        return [p.x, p.y, tw.linear.x, tw.linear.y, tw.angular.z]
    if type_str == 'sensor_msgs/msg/CameraInfo':
        return [float(msg.width), float(msg.height)]
    return None


def _segment_boundaries(info, total_duration_s):
    """Yield (x_seconds, label) for segment end boundaries."""
    segs = info.get('segments', []) or []
    fps = float(info.get('fps', 15))
    if not segs or fps <= 0:
        return []
    last_end_frame = segs[-1]['frame_duration'][1]
    scale = total_duration_s / max(1, last_end_frame)
    boundaries = []
    for seg in segs[:-1]:
        end_frame = seg['frame_duration'][1]
        boundaries.append((end_frame * scale,
                           seg.get('primitive_description', '')))
    return boundaries


def plot(episode_dir: Path, out_path: Path = None, show: bool = False):
    info_path = episode_dir / 'episode_info.json'
    info = json.loads(info_path.read_text()) if info_path.exists() else {}

    by_topic = _collect(episode_dir)
    if not by_topic:
        print('No topics found.')
        return 2

    # Normalize timestamps to seconds from t0 for readability.
    all_times = [t for e in by_topic.values() for t in e['times']]
    if not all_times:
        print('No messages found.')
        return 2
    t0 = min(all_times)
    t_end = max(all_times)
    duration_s = (t_end - t0) / 1e9

    # Split topics into (plottable, event-only).
    plot_topics = {
        n: e for n, e in by_topic.items() if e['values']
    }
    event_topics = {
        n: e for n, e in by_topic.items() if not e['values']
    }

    n_plot = len(plot_topics)
    n_rows = n_plot + (1 if event_topics else 0)
    fig, axes = plt.subplots(
        n_rows, 1,
        figsize=(14, max(2.0 * n_rows, 4)),
        sharex=True,
        squeeze=False,
    )
    axes = [ax[0] for ax in axes]

    boundaries = _segment_boundaries(info, duration_s)

    def draw_boundaries(ax):
        for x, label in boundaries:
            ax.axvline(x, color='red', linestyle='--', linewidth=0.8, alpha=0.7)
        # Label on top subplot only handled outside.

    # Value plots.
    for idx, (topic, entry) in enumerate(sorted(plot_topics.items())):
        ax = axes[idx]
        times = [(t - t0) / 1e9 for t in entry['times']]
        values = entry['values']
        # Ensure same length.
        n = min(len(times), len(values))
        times = times[:n]
        values = values[:n]
        # Transpose to per-dim traces.
        dims = list(zip(*values)) if values else []
        for d_idx, dim in enumerate(dims):
            ax.plot(times, dim, linewidth=0.7, alpha=0.85,
                    label=f'[{d_idx}]')
        short = topic.split('/')[-1] or topic
        ax.set_ylabel(short, fontsize=8)
        ax.tick_params(axis='both', labelsize=7)
        ax.grid(True, linestyle=':', alpha=0.4)
        ax.set_title(topic, fontsize=8, loc='left')
        if len(dims) <= 10:
            ax.legend(loc='upper right', fontsize=6, ncol=min(len(dims), 5))
        draw_boundaries(ax)

    # Event plot for remaining topics.
    if event_topics:
        ax = axes[-1]
        topic_names = sorted(event_topics.keys())
        offsets = list(range(len(topic_names)))
        for y, topic in zip(offsets, topic_names):
            times = [(t - t0) / 1e9 for t in event_topics[topic]['times']]
            ax.scatter(times, [y] * len(times), s=2, marker='|')
        ax.set_yticks(offsets)
        ax.set_yticklabels(topic_names, fontsize=7)
        ax.set_title('Event stream (non-plottable topics)',
                     fontsize=8, loc='left')
        ax.grid(True, axis='x', linestyle=':', alpha=0.4)
        draw_boundaries(ax)

    axes[-1].set_xlabel('time since first message [s]')
    fig.suptitle(f'{episode_dir.name}  ({duration_s:.2f}s, '
                 f'{len(by_topic)} topics)', fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    if out_path:
        fig.savefig(out_path, dpi=140)
        print(f'Saved: {out_path}')
    if show:
        plt.show()
    return 0


def main(argv):
    ap = argparse.ArgumentParser(
        description='Visualize all topics of a merged episode.')
    ap.add_argument('episode_dir', type=Path)
    ap.add_argument('--save', type=Path, default=None,
                    help='Save figure to this path (PNG/PDF).')
    ap.add_argument('--show', action='store_true',
                    help='Open an interactive window.')
    args = ap.parse_args(argv)

    if not args.show and args.save is None:
        # Default: save next to episode dir.
        args.save = args.episode_dir / 'episode_timeseries.png'
    if not args.show:
        matplotlib.use('Agg')
    return plot(args.episode_dir, out_path=args.save, show=args.show)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
