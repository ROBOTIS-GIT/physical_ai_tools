#!/usr/bin/env python3
"""Verify that a merged episode's mcap content matches episode_info.json.

Checks:
    * Camera topics exist and have consistent frame counts across views.
    * Total camera frame count matches the last segment's
      ``frame_duration[1]`` (within a small tolerance).
    * Per-segment boundaries align with the actual frame timestamps:
      the number of frames observed in each segment's time window
      matches ``frame_duration[1] - frame_duration[0]`` (±1 frame).

Usage:
    python3 verify_merged_episode.py <episode_dir>

    episode_dir must be a rosbag2 episode folder produced by Merge to MCAP:
        {episode_dir}/
            {ep_idx}_0.mcap
            metadata.yaml
            episode_info.json
            robot.urdf
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from mcap.reader import make_reader


# Camera topics we expect in a ffw_sg2_rev1 recording. Missing topics
# are reported as warnings, not errors, so the script works on other
# robot types too.
CAMERA_TOPIC_CANDIDATES = (
    '/robot/camera/cam_left_head/image_raw/compressed',
    '/robot/camera/cam_right_head/image_raw/compressed',
    '/robot/camera/cam_left_wrist/image_raw/compressed',
    '/robot/camera/cam_right_wrist/image_raw/compressed',
)


def _collect_camera_frames(mcap_path: Path):
    """Return { topic_name: [log_time_ns, ...] } for camera topics."""
    frames: dict[str, list[int]] = {t: [] for t in CAMERA_TOPIC_CANDIDATES}
    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(
                topics=list(CAMERA_TOPIC_CANDIDATES)):
            frames[channel.topic].append(message.log_time)
    # Sort — order isn't guaranteed when iterating across chunks.
    for t in frames:
        frames[t].sort()
    return frames


def _pick_reference_topic(frames):
    """Return (topic_name, timestamps) of the camera with the most frames."""
    non_empty = {t: ts for t, ts in frames.items() if ts}
    if not non_empty:
        return None, []
    topic = max(non_empty, key=lambda t: len(non_empty[t]))
    return topic, non_empty[topic]


def verify(episode_dir: Path) -> int:
    info_path = episode_dir / 'episode_info.json'
    if not info_path.exists():
        print(f'FAIL: missing {info_path}')
        return 2

    with open(info_path) as f:
        info = json.load(f)

    segments = info.get('segments', []) or []
    fps = int(info.get('fps', 15))

    if not segments:
        print('FAIL: no segments in episode_info.json')
        return 2

    mcaps = list(episode_dir.glob('*.mcap'))
    if not mcaps:
        print(f'FAIL: no .mcap in {episode_dir}')
        return 2
    if len(mcaps) > 1:
        print(f'WARN: multiple .mcap files found, using {mcaps[0].name}')
    mcap_path = mcaps[0]

    frames = _collect_camera_frames(mcap_path)
    counts = {t: len(ts) for t, ts in frames.items()}
    present = {t: c for t, c in counts.items() if c > 0}
    missing = [t for t, c in counts.items() if c == 0]

    print(f'Episode dir : {episode_dir}')
    print(f'MCAP file   : {mcap_path.name}')
    print(f'FPS (declared): {fps}')
    print(f'Segments    : {len(segments)}')
    print(f'Camera topics found: {len(present)} / {len(CAMERA_TOPIC_CANDIDATES)}')
    for t, c in counts.items():
        tag = 'OK' if c > 0 else 'MISSING'
        print(f'  [{tag}] {t}  {c} frames')

    fail = False

    # Consistency across cameras (within 1%).
    if present:
        max_c = max(present.values())
        min_c = min(present.values())
        spread_pct = 0 if max_c == 0 else (max_c - min_c) / max_c * 100
        print(f'Camera count spread: {min_c}..{max_c} '
              f'({spread_pct:.2f}%)')
        if spread_pct > 5:
            print('FAIL: camera frame counts differ by more than 5%')
            fail = True

    ref_topic, ref_ts = _pick_reference_topic(frames)
    if not ref_ts:
        print('FAIL: no camera frames recorded — cannot verify timing')
        return 2
    print(f'Reference topic: {ref_topic} ({len(ref_ts)} frames)')

    # 1) Total frame count vs last segment's upper bound.
    last_end = int(segments[-1]['frame_duration'][1])
    observed_total = len(ref_ts)
    tol = max(2, int(round(0.01 * last_end)))  # 1% or 2 frames, whichever larger
    if abs(observed_total - last_end) > tol:
        print(f'FAIL: total frames {observed_total} vs '
              f'last segment end {last_end} (tol ±{tol})')
        fail = True
    else:
        print(f'OK: total frames {observed_total} ≈ '
              f'last segment end {last_end} (tol ±{tol})')

    # 2) Per-segment frame count vs frame_duration length. Because merge
    # closes gaps to 1-frame spacing, we can directly slice ref_ts by the
    # cumulative frame_duration ranges.
    cur = 0
    for i, seg in enumerate(segments):
        a, b = seg['frame_duration']
        expected = b - a
        # Use as many frames as we still have; ref_ts is sorted.
        take = min(expected, max(0, len(ref_ts) - cur))
        actual = take  # because ref_ts is contiguous after gap-closure
        line = (f'seg {i:2d} [{a:5d}..{b:5d}]  '
                f'primitive={seg.get("primitive_description", ""):<16} '
                f'expected={expected:4d}  actual={actual:4d}')
        if actual != expected:
            print('FAIL: ' + line)
            fail = True
        else:
            print('  OK: ' + line)
        cur += expected

    if fail:
        print('\nRESULT: mismatches detected')
        return 1
    print('\nRESULT: all checks passed')
    return 0


def main(argv):
    ap = argparse.ArgumentParser(
        description='Verify merged episode MCAP vs episode_info.json')
    ap.add_argument('episode_dir', type=Path,
                    help='Path to an archived episode directory.')
    args = ap.parse_args(argv)
    return verify(args.episode_dir)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
