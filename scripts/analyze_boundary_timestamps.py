#!/usr/bin/env python3
"""Deep analysis of per-topic timestamps around segment boundaries.

For each topic, prints:
- Last N messages before boundary & first N messages after boundary
- Exact ns timestamps and deltas
- Whether timestamps go BACKWARDS (non-monotonic per-topic)
- Inter-topic phase alignment at boundaries

Usage:
    python3 analyze_boundary_timestamps.py <episode_dir> [--context 5]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from mcap.reader import make_reader


def _collect(mcap_path: Path):
    by_topic: dict[str, list[int]] = {}
    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            by_topic.setdefault(channel.topic, []).append(message.log_time)
    for ts in by_topic.values():
        ts.sort()
    return by_topic


def _find_boundary_index(ts, boundary_ns):
    """Return index of first timestamp > boundary_ns."""
    for i, t in enumerate(ts):
        if t > boundary_ns:
            return i
    return len(ts)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('episode_dir', type=Path)
    ap.add_argument('--context', type=int, default=5,
                    help='Messages to show before/after boundary.')
    args = ap.parse_args(argv)

    info_path = args.episode_dir / 'episode_info.json'
    info = json.loads(info_path.read_text())
    stitch_ns = info.get('stitch_times_ns', {}) or {}
    segments = info.get('segments', []) or []

    mcap_files = list(args.episode_dir.glob('*.mcap'))
    if not mcap_files:
        print('No .mcap file found')
        return 2

    by_topic = _collect(mcap_files[0])
    all_ts = [t for ts in by_topic.values() for t in ts]
    t0 = min(all_ts)
    t_end = max(all_ts)
    duration_s = (t_end - t0) / 1e9

    print(f'Episode: {args.episode_dir}')
    print(f'MCAP: {mcap_files[0].name}')
    print(f'Duration: {duration_s:.3f}s')
    print(f't0 (ns): {t0}')
    print(f'Topics: {len(by_topic)}')
    print(f'Segments: {len(segments)}')
    print(f'Stitch data for: {len(stitch_ns)} topics')
    print()

    num_boundaries = len(segments) - 1
    for b_idx in range(num_boundaries):
        print(f'{"=" * 90}')
        print(f'BOUNDARY {b_idx} (segment {b_idx} -> {b_idx + 1})')
        print(f'{"=" * 90}')

        topic_stitch = {}
        for topic, times in stitch_ns.items():
            if b_idx < len(times):
                topic_stitch[topic] = int(times[b_idx])

        if topic_stitch:
            stitch_min = min(topic_stitch.values())
            stitch_max = max(topic_stitch.values())
            spread = (stitch_max - stitch_min) / 1e6
            print(f'Stitch range: {(stitch_min-t0)/1e9:.6f}s .. '
                  f'{(stitch_max-t0)/1e9:.6f}s  (spread: {spread:.2f} ms)')
        else:
            stitch_min = None
        print()

        for topic in sorted(by_topic):
            ts = by_topic[topic]
            stitch_t = topic_stitch.get(topic)

            if stitch_t is None:
                if stitch_min is not None:
                    stitch_t = stitch_min
                else:
                    last_end_frame = segments[-1]['frame_duration'][1]
                    scale = (t_end - t0) / max(1, last_end_frame)
                    stitch_t = t0 + int(
                        segments[b_idx]['frame_duration'][1] * scale)

            bi = _find_boundary_index(ts, stitch_t - 1)
            lo = max(0, bi - args.context)
            hi = min(len(ts), bi + args.context)

            print(f'--- {topic} ({len(ts)} msgs, '
                  f'stitch@{(stitch_t-t0)/1e9:.6f}s) ---')

            prev_t = None
            for i in range(lo, hi):
                t = ts[i]
                rel = (t - t0) / 1e9
                delta_ms = (t - prev_t) / 1e6 if prev_t is not None else 0
                marker = ''
                if prev_t is not None and prev_t <= stitch_t <= t:
                    marker = ' <-- STITCH'
                if prev_t is not None and t < prev_t:
                    marker += ' *** NON-MONOTONIC ***'
                if delta_ms > 200:
                    marker += f' *** GAP {delta_ms:.1f}ms ***'
                elif delta_ms > 100:
                    marker += f' (large: {delta_ms:.1f}ms)'
                print(f'  [{i:5d}] t={rel:12.6f}s  '
                      f'delta={delta_ms:8.2f}ms{marker}')
                prev_t = t
            print()

        # Cross-topic alignment
        print(f'--- Cross-topic phase alignment at boundary {b_idx} ---')
        rows = []
        for topic in sorted(by_topic):
            ts = by_topic[topic]
            stitch_t = topic_stitch.get(topic, stitch_min or 0)
            bi = _find_boundary_index(ts, stitch_t - 1)
            last_before = ts[bi - 1] if bi > 0 else None
            first_after = ts[bi] if bi < len(ts) else None
            gap_ms = ((first_after - last_before) / 1e6
                      if first_after and last_before else None)
            rows.append((topic, last_before, first_after, gap_ms))

        if rows:
            ref_first = min(
                r[2] for r in rows if r[2] is not None)
            print(f'  {"topic":60s}  {"last_bef(s)":>12s}  '
                  f'{"first_aft(s)":>12s}  {"gap_ms":>8s}  {"phase_ms":>8s}')
            for topic, lb, fa, gap in rows:
                lb_s = f'{(lb-t0)/1e9:.6f}' if lb else 'n/a'
                fa_s = f'{(fa-t0)/1e9:.6f}' if fa else 'n/a'
                gap_s = f'{gap:.2f}' if gap is not None else 'n/a'
                phase = f'{(fa-ref_first)/1e6:.2f}' if fa else 'n/a'
                print(f'  {topic:60s}  {lb_s:>12s}  '
                      f'{fa_s:>12s}  {gap_s:>8s}  {phase:>8s}')
        print()


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
