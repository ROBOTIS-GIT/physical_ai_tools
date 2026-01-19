#!/usr/bin/env python3
"""
Rosbag Timing Visualization Tool

Generates timeline plots showing message arrival patterns and gaps in rosbag recordings.
Provides comprehensive analysis of topic synchronization, delays, and anomalies.
"""

import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import json
from rosbags.rosbag2 import Reader


@dataclass
class TopicStatistics:
    """Statistics for a single topic."""
    topic: str
    msg_type: str
    message_count: int
    duration: float
    frequency: float
    first_timestamp: float
    last_timestamp: float
    min_gap_ms: float
    max_gap_ms: float
    avg_gap_ms: float
    std_dev_gap_ms: float
    gaps_over_threshold: List[Tuple[float, float, float]]


class TimingVisualizer:
    """Visualize rosbag message timing with comprehensive analysis."""

    def __init__(
        self,
        bag_path: str,
        segment_duration: float = 5.0,
        gap_threshold_ms: float = 100.0,
        output_dir: Optional[str] = None
    ):
        """
        Initialize visualizer.

        Args:
            bag_path: Path to rosbag directory
            segment_duration: Timeline segment duration in seconds
            gap_threshold_ms: Threshold for flagging gaps as anomalies
            output_dir: Output directory for files (default: bag directory)
        """
        self.bag_path = Path(bag_path)
        self.segment_duration = segment_duration
        self.gap_threshold_ms = gap_threshold_ms
        self.topic_times = {}  # Dict[str, List[float]]
        self.topic_types = {}  # Dict[str, str]
        self.statistics = {}  # Dict[str, TopicStatistics]

        if output_dir:
            self.output_dir = Path(output_dir)
            self.output_dir.mkdir(parents=True, exist_ok=True)
        else:
            self.output_dir = self.bag_path.parent

    def analyze_bag(self):
        """Extract all message timestamps from bag."""
        print(f"Analyzing bag: {self.bag_path}")

        with Reader(str(self.bag_path)) as reader:
            # Get topic types and collect timestamps
            for connection in reader.connections:
                self.topic_types[connection.topic] = connection.msgtype

            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic
                time_sec = timestamp / 1e9  # ns to seconds

                if topic not in self.topic_times:
                    self.topic_times[topic] = []
                self.topic_times[topic].append(time_sec)

        # Normalize to start at 0
        if self.topic_times:
            all_times = [t for times in self.topic_times.values() for t in times]
            min_time = min(all_times)

            for topic in self.topic_times:
                self.topic_times[topic] = [t - min_time for t in self.topic_times[topic]]

        print(f"Found {len(self.topic_times)} topics")
        for topic, times in self.topic_times.items():
            print(f"  {topic}: {len(times)} messages")

    def compute_statistics(self) -> Dict[str, TopicStatistics]:
        """Compute detailed statistics for each topic."""
        stats = {}

        for topic in sorted(self.topic_times.keys()):
            times = sorted(self.topic_times[topic])

            if len(times) == 0:
                continue

            duration = times[-1] - times[0]
            frequency = len(times) / duration if duration > 0 else 0

            # Compute gaps
            gaps = []
            for i in range(1, len(times)):
                gap_ms = (times[i] - times[i - 1]) * 1000
                gaps.append(gap_ms)

            min_gap = min(gaps) if gaps else 0
            max_gap = max(gaps) if gaps else 0
            avg_gap = np.mean(gaps) if gaps else 0
            std_dev = np.std(gaps) if gaps else 0

            # Find gaps over threshold
            large_gaps = []
            for i in range(1, len(times)):
                gap_ms = (times[i] - times[i - 1]) * 1000
                if gap_ms > self.gap_threshold_ms:
                    large_gaps.append((times[i - 1], times[i], gap_ms))

            stats[topic] = TopicStatistics(
                topic=topic,
                msg_type=self.topic_types.get(topic, "unknown"),
                message_count=len(times),
                duration=duration,
                frequency=frequency,
                first_timestamp=times[0],
                last_timestamp=times[-1],
                min_gap_ms=min_gap,
                max_gap_ms=max_gap,
                avg_gap_ms=avg_gap,
                std_dev_gap_ms=std_dev,
                gaps_over_threshold=large_gaps
            )

        self.statistics = stats
        return stats

    def detect_gaps(self, topic: str) -> List[Tuple[float, float, float]]:
        """Detect gaps in message stream.

        Returns:
            List of (gap_start_time, gap_end_time, gap_ms)
        """
        times = sorted(self.topic_times[topic])
        gaps = []

        for i in range(1, len(times)):
            gap_ms = (times[i] - times[i - 1]) * 1000
            if gap_ms > self.gap_threshold_ms:
                gaps.append((times[i - 1], times[i], gap_ms))

        return gaps

    def get_color_for_topic(self, topic: str, msg_type: str, color_index: int) -> str:
        """Assign color based on message type."""
        color_map = {
            "image": "#1f77b4",      # Blue
            "joint_state": "#2ca02c", # Green
            "tf": "#ff7f0e",          # Orange
            "odometry": "#9467bd",    # Purple
            "laser_scan": "#d62728",  # Red
            "camera_info": "#17becf", # Cyan
            "imu": "#bcbd22",         # Yellow-green
            "twist": "#8c564b",       # Brown
        }

        # Check message type
        for key, color in color_map.items():
            if key.lower() in msg_type.lower():
                return color

        # Default color palette
        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
                  "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]
        return colors[color_index % len(colors)]

    def visualize(self, output_path: Optional[str] = None) -> str:
        """Generate comprehensive timeline visualization.

        Args:
            output_path: Output PNG file path (optional)

        Returns:
            Path to generated PNG file
        """
        if not self.topic_times:
            self.analyze_bag()

        if not self.statistics:
            self.compute_statistics()

        # Calculate figure dimensions
        all_times = [t for times in self.topic_times.values() for t in times]
        max_time = max(all_times)
        num_segments = int(np.ceil(max_time / self.segment_duration))
        num_topics = len(self.topic_times)

        # Determine output path
        if output_path is None:
            bag_name = self.bag_path.name if self.bag_path.is_dir() else self.bag_path.stem
            output_path = str(self.output_dir / f"{bag_name}_timing.png")

        # Create figure with subplots for timeline + statistics
        fig = plt.figure(figsize=(20, 2.5 * num_segments + 3))
        gs = gridspec.GridSpec(num_segments + 1, 1, height_ratios=[2.5]*num_segments + [1])

        # Assign colors
        topics = sorted(self.topic_times.keys())
        color_map = {}
        for idx, topic in enumerate(topics):
            msg_type = self.topic_types.get(topic, "")
            color_map[topic] = self.get_color_for_topic(topic, msg_type, idx)

        # Plot each timeline segment
        axes = []
        for seg_idx in range(num_segments):
            ax = fig.add_subplot(gs[seg_idx, 0])
            axes.append(ax)

            seg_start = seg_idx * self.segment_duration
            seg_end = (seg_idx + 1) * self.segment_duration

            for topic_idx, topic in enumerate(topics):
                times = self.topic_times[topic]

                # Filter times for this segment
                seg_times = [t for t in times if seg_start <= t < seg_end]

                if seg_times:
                    # Plot messages as scatter points
                    ax.scatter(
                        seg_times,
                        [topic_idx] * len(seg_times),
                        c=[color_map[topic]],
                        s=30,
                        alpha=0.8,
                        marker='|',
                        linewidths=2
                    )

                # Detect and mark gaps (only once)
                if seg_idx == 0:
                    gaps = self.detect_gaps(topic)
                    for gap_start, gap_end, gap_ms in gaps:
                        if seg_start <= gap_start < seg_end:
                            # Mark gap with red vertical line
                            ax.axvline(
                                gap_start,
                                color='#d62728',
                                linestyle='--',
                                alpha=0.6,
                                linewidth=2
                            )
                            # Add gap annotation
                            ax.text(
                                gap_start,
                                topic_idx + 0.3,
                                f"{gap_ms:.1f}ms",
                                fontsize=7,
                                color='#d62728',
                                fontweight='bold',
                                rotation=0
                            )

            # Format segment
            ax.set_xlim(seg_start, seg_end)
            ax.set_ylim(-0.5, num_topics - 0.5)
            ax.set_yticks(range(num_topics))
            ax.set_yticklabels(topics, fontsize=9)
            ax.set_ylabel(f"Segment {seg_idx + 1}\n({seg_start:.1f}-{seg_end:.1f}s)",
                         fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.3, axis='x', linestyle=':')
            ax.set_axisbelow(True)

            # Add 10ms grid lines
            for ms in range(0, int((seg_end - seg_start) * 1000) + 1, 10):
                ax.axvline(seg_start + ms / 1000, color='gray', alpha=0.1, linewidth=0.5)

        # Set xlabel on bottom timeline
        axes[-1].set_xlabel("Time (seconds)", fontsize=12, fontweight='bold')

        # Statistics panel
        ax_stats = fig.add_subplot(gs[num_segments, 0])
        ax_stats.axis('off')

        # Create statistics text
        stats_text = "TIMING STATISTICS\n" + "=" * 80 + "\n"
        for topic in topics:
            stat = self.statistics.get(topic)
            if stat:
                msg_count = stat.message_count
                freq = stat.frequency
                min_g = stat.min_gap_ms
                max_g = stat.max_gap_ms
                avg_g = stat.avg_gap_ms
                std_g = stat.std_dev_gap_ms
                num_anomalies = len(stat.gaps_over_threshold)

                anomaly_marker = " [ANOMALY!]" if num_anomalies > 0 else ""
                stats_text += (
                    f"\n{topic}\n"
                    f"  Messages: {msg_count} | Frequency: {freq:.2f}Hz "
                    f"| Gap (min/avg/max): {min_g:.2f}/{avg_g:.2f}/{max_g:.2f}ms "
                    f"(σ={std_g:.2f}){anomaly_marker}\n"
                )
                if num_anomalies > 0:
                    stats_text += f"  ⚠️  {num_anomalies} gaps over {self.gap_threshold_ms}ms threshold\n"

        ax_stats.text(
            0.02, 0.95,
            stats_text,
            transform=ax_stats.transAxes,
            fontsize=9,
            verticalalignment='top',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        )

        # Title
        fig.suptitle(
            f"Rosbag Timing Visualization: {self.bag_path.name}\n"
            f"Timeline with {self.segment_duration}s segments | Red dashed lines mark gaps > {self.gap_threshold_ms}ms",
            fontsize=14,
            fontweight='bold',
            y=0.98
        )

        # Legend
        legend_elements = []
        for topic in topics:
            stat = self.statistics.get(topic)
            if stat:
                label = f"{topic} ({stat.message_count} msgs @ {stat.frequency:.1f}Hz)"
            else:
                label = f"{topic}"
            legend_elements.append(
                mpatches.Patch(
                    facecolor=color_map[topic],
                    edgecolor='black',
                    label=label
                )
            )

        fig.legend(
            handles=legend_elements,
            loc='lower center',
            ncol=min(4, len(topics)),
            fontsize=9,
            framealpha=0.95,
            bbox_to_anchor=(0.5, -0.02)
        )

        # Save
        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()

        print(f"\nVisualization saved: {output_path}")
        return output_path

    def generate_report(self, output_path: Optional[str] = None) -> str:
        """Generate detailed JSON report of timing analysis.

        Args:
            output_path: Output JSON file path (optional)

        Returns:
            Path to generated JSON file
        """
        if not self.statistics:
            self.compute_statistics()

        # Determine output path
        if output_path is None:
            bag_name = self.bag_path.name if self.bag_path.is_dir() else self.bag_path.stem
            output_path = str(self.output_dir / f"{bag_name}_timing_report.json")

        # Build report
        report = {
            "bag_path": str(self.bag_path),
            "timestamp_ms": int(np.ceil(max([t for times in self.topic_times.values() for t in times]) * 1000)) if self.topic_times else 0,
            "total_topics": len(self.topic_times),
            "segment_duration_s": self.segment_duration,
            "gap_threshold_ms": self.gap_threshold_ms,
            "topics": {}
        }

        for topic in sorted(self.topic_times.keys()):
            stat = self.statistics.get(topic)
            if stat:
                report["topics"][topic] = {
                    "message_type": stat.msg_type,
                    "message_count": stat.message_count,
                    "duration_s": stat.duration,
                    "frequency_hz": stat.frequency,
                    "first_timestamp_s": stat.first_timestamp,
                    "last_timestamp_s": stat.last_timestamp,
                    "gap_statistics": {
                        "min_ms": stat.min_gap_ms,
                        "max_ms": stat.max_gap_ms,
                        "avg_ms": stat.avg_gap_ms,
                        "std_dev_ms": stat.std_dev_gap_ms
                    },
                    "anomalies": {
                        "count": len(stat.gaps_over_threshold),
                        "threshold_ms": self.gap_threshold_ms,
                        "gaps": [
                            {
                                "start_s": gap_start,
                                "end_s": gap_end,
                                "duration_ms": gap_ms
                            }
                            for gap_start, gap_end, gap_ms in stat.gaps_over_threshold[:10]
                        ]
                    }
                }

        # Save report
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)

        print(f"Report saved: {output_path}")
        return output_path

    def print_summary(self):
        """Print text summary of findings."""
        if not self.statistics:
            self.compute_statistics()

        print("\n" + "=" * 100)
        print("TIMING ANALYSIS SUMMARY".center(100))
        print("=" * 100)

        for topic in sorted(self.topic_times.keys()):
            stat = self.statistics.get(topic)
            if not stat:
                continue

            print(f"\nTopic: {topic}")
            print(f"  Type: {stat.msg_type}")
            print(f"  Messages: {stat.message_count} over {stat.duration:.3f}s")
            print(f"  Frequency: {stat.frequency:.2f} Hz")
            print(f"  Gap Statistics (ms): min={stat.min_gap_ms:.3f}, " +
                  f"avg={stat.avg_gap_ms:.3f}, max={stat.max_gap_ms:.3f}, " +
                  f"σ={stat.std_dev_gap_ms:.3f}")

            if stat.gaps_over_threshold:
                print(f"  ⚠️  ANOMALIES DETECTED: {len(stat.gaps_over_threshold)} gaps > {self.gap_threshold_ms}ms")
                for gap_start, gap_end, gap_ms in stat.gaps_over_threshold[:5]:
                    print(f"     Gap at {gap_start:.3f}s: {gap_ms:.1f}ms")
                if len(stat.gaps_over_threshold) > 5:
                    print(f"     ... and {len(stat.gaps_over_threshold) - 5} more")
            else:
                print(f"  ✓ No gaps > {self.gap_threshold_ms}ms detected")

        print("\n" + "=" * 100)


def main():
    parser = argparse.ArgumentParser(
        description="Visualize rosbag message timing with comprehensive analysis",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  python3 visualize_timing.py /workspace/test_bag/episode_0

  # Custom output path
  python3 visualize_timing.py /workspace/test_bag/episode_0 -o timeline.png

  # Custom segment duration (2 seconds)
  python3 visualize_timing.py /workspace/test_bag/episode_0 -s 2.0

  # Custom gap threshold (50ms)
  python3 visualize_timing.py /workspace/test_bag/episode_0 -g 50.0

  # With report generation
  python3 visualize_timing.py /workspace/test_bag/episode_0 -r report.json
        """
    )
    parser.add_argument("bag_path", help="Path to rosbag directory")
    parser.add_argument("-o", "--output", help="Output PNG file path")
    parser.add_argument("-r", "--report", help="Output JSON report file path")
    parser.add_argument("-d", "--output-dir", help="Output directory for all files")
    parser.add_argument("-s", "--segment-duration", type=float, default=5.0,
                       help="Duration of each timeline segment (seconds, default=5.0)")
    parser.add_argument("-g", "--gap-threshold", type=float, default=100.0,
                       help="Gap detection threshold (milliseconds, default=100.0)")

    args = parser.parse_args()

    # Create visualizer
    viz = TimingVisualizer(
        args.bag_path,
        segment_duration=args.segment_duration,
        gap_threshold_ms=args.gap_threshold,
        output_dir=args.output_dir
    )

    # Analyze bag
    viz.analyze_bag()
    viz.compute_statistics()
    viz.print_summary()

    # Generate visualization
    viz.visualize(args.output)

    # Generate report
    if args.report:
        viz.generate_report(args.report)
    else:
        viz.generate_report()


if __name__ == "__main__":
    main()
