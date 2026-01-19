#!/usr/bin/env python3
"""
Detailed visualization for CompressedImage recording test
Similar to original_timing.png style but optimized for ImageMetadata
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from pathlib import Path
import struct
from rosbags.rosbag2 import Reader
import json

def extract_all_timestamps(bag_path):
    """Extract timestamps from all topics in recorded bag"""
    topics_data = {}

    with Reader(Path(bag_path)) as reader:
        for conn in reader.connections:
            topic = conn.topic
            msgtype = conn.msgtype
            topics_data[topic] = {
                'type': msgtype,
                'timestamps': [],
                'bag_times': []
            }

            for connection, bag_time, rawdata in reader.messages():
                if connection.id == conn.id:
                    # Extract header timestamp if available
                    try:
                        offset = 4
                        sec = struct.unpack('<i', rawdata[offset:offset+4])[0]
                        nanosec = struct.unpack('<I', rawdata[offset+4:offset+8])[0]
                        timestamp = sec + nanosec / 1e9
                    except:
                        timestamp = bag_time / 1e9

                    topics_data[topic]['timestamps'].append(timestamp)
                    topics_data[topic]['bag_times'].append(bag_time / 1e9)

    return topics_data

def create_detailed_visualization(bag_path, output_path):
    """Create detailed visualization similar to original_timing.png"""

    print(f"Analyzing bag: {bag_path}")
    topics_data = extract_all_timestamps(bag_path)

    # Filter to only relevant topics
    relevant_topics = {k: v for k, v in topics_data.items() if v['timestamps']}

    print(f"Found {len(relevant_topics)} topics")
    for topic, data in relevant_topics.items():
        print(f"  {topic}: {len(data['timestamps'])} messages")

    # Create figure with subplots
    n_topics = len(relevant_topics)
    fig = plt.figure(figsize=(16, max(12, n_topics * 2)))

    # Create grid spec for flexible subplot sizing
    gs = gridspec.GridSpec(n_topics + 1, 2,
                          height_ratios=[3] * n_topics + [1],
                          width_ratios=[3, 1],
                          hspace=0.4, wspace=0.3)

    fig.suptitle(f'CompressedImage Recording Test - Detailed Analysis\n' +
                 f'Bag: {Path(bag_path).name} | Duration: 10.26s | Topics: {n_topics}',
                 fontsize=14, fontweight='bold')

    colors = plt.cm.tab10(np.linspace(0, 1, n_topics))

    # Summary statistics
    summary_text = "="*70 + "\n"
    summary_text += "TIMING ANALYSIS SUMMARY\n"
    summary_text += "="*70 + "\n\n"

    for idx, (topic, data) in enumerate(sorted(relevant_topics.items())):
        timestamps = data['timestamps']
        msgtype = data['type']

        if len(timestamps) < 2:
            continue

        # Calculate intervals
        intervals = []
        for i in range(1, len(timestamps)):
            gap_ms = (timestamps[i] - timestamps[i-1]) * 1000
            intervals.append(gap_ms)

        # Statistics
        avg_gap = np.mean(intervals)
        min_gap = np.min(intervals)
        max_gap = np.max(intervals)
        std_gap = np.std(intervals)
        freq = 1000.0 / avg_gap if avg_gap > 0 else 0

        # Timeline plot (left)
        ax_timeline = fig.add_subplot(gs[idx, 0])

        # Normalize timestamps to start from 0
        start_time = timestamps[0]
        rel_timestamps = [(t - start_time) for t in timestamps]

        # Plot as vertical lines
        ax_timeline.vlines(rel_timestamps, 0, 1, colors=colors[idx], linewidth=1.5, alpha=0.7)
        ax_timeline.set_ylim(-0.1, 1.1)
        ax_timeline.set_ylabel(topic.split('/')[-1] if '/' in topic else topic,
                              fontsize=9, rotation=0, ha='right', va='center')
        ax_timeline.set_xlim(0, max(rel_timestamps) if rel_timestamps else 1)
        ax_timeline.grid(True, alpha=0.3, axis='x')
        ax_timeline.set_yticks([])

        if idx == n_topics - 1:
            ax_timeline.set_xlabel('Time (s)', fontsize=10)
        else:
            ax_timeline.set_xticklabels([])

        # Interval histogram (right)
        ax_hist = fig.add_subplot(gs[idx, 1])
        ax_hist.hist(intervals, bins=20, color=colors[idx], alpha=0.7, edgecolor='black')
        ax_hist.axvline(avg_gap, color='red', linestyle='--', linewidth=2, label=f'Avg: {avg_gap:.1f}ms')
        ax_hist.set_xlabel('Interval (ms)', fontsize=9)
        ax_hist.set_ylabel('Count', fontsize=9)
        ax_hist.legend(fontsize=8)
        ax_hist.grid(True, alpha=0.3)

        # Add to summary
        summary_text += f"Topic: {topic}\n"
        summary_text += f"  Type: {msgtype}\n"
        summary_text += f"  Messages: {len(timestamps)}\n"
        summary_text += f"  Frequency: {freq:.2f} Hz\n"
        summary_text += f"  Gap Statistics (ms): min={min_gap:.3f}, avg={avg_gap:.3f}, max={max_gap:.3f}, σ={std_gap:.3f}\n"

        # Check for anomalies
        if 'metadata' in topic.lower() or 'compressed' in topic.lower():
            threshold = 100.0  # ms
            anomalies = [g for g in intervals if g > threshold]
            if anomalies:
                summary_text += f"  ⚠️  ANOMALIES DETECTED: {len(anomalies)} gaps > {threshold}ms\n"
            else:
                summary_text += f"  ✓ No gaps > {threshold}ms detected\n"

        summary_text += "\n"

    # Add summary text at bottom
    ax_summary = fig.add_subplot(gs[n_topics, :])
    ax_summary.axis('off')
    ax_summary.text(0.05, 0.95, summary_text,
                   transform=ax_summary.transAxes,
                   fontsize=8, verticalalignment='top',
                   fontfamily='monospace',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

    # Save
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"\n✅ Visualization saved: {output_path}")

    # Save summary to text file
    summary_file = output_path.replace('.png', '_summary.txt')
    with open(summary_file, 'w') as f:
        f.write(summary_text)
    print(f"✅ Summary saved: {summary_file}")

    # Save to JSON
    json_file = output_path.replace('.png', '_report.json')
    report_data = {}
    for topic, data in relevant_topics.items():
        if len(data['timestamps']) > 1:
            timestamps = data['timestamps']
            intervals = [(timestamps[i] - timestamps[i-1]) * 1000
                        for i in range(1, len(timestamps))]
            report_data[topic] = {
                'type': data['type'],
                'message_count': len(timestamps),
                'duration_sec': timestamps[-1] - timestamps[0],
                'frequency_hz': 1000.0 / np.mean(intervals) if intervals else 0,
                'interval_stats': {
                    'min_ms': float(np.min(intervals)),
                    'avg_ms': float(np.mean(intervals)),
                    'max_ms': float(np.max(intervals)),
                    'std_ms': float(np.std(intervals))
                }
            }

    with open(json_file, 'w') as f:
        json.dump(report_data, f, indent=2)
    print(f"✅ JSON report saved: {json_file}")

if __name__ == '__main__':
    create_detailed_visualization(
        '/workspace/test_final/recording',
        '/workspace/test_final/recorded_timing_detailed.png'
    )
