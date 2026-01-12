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
#
# Author: AI Assistant (Sisyphus)

"""
ROSbag Data Quality Analyzer.

Analyzes timestamp gaps and data drops in ROSbag recordings
to ensure data quality before conversion to LeRobot format.

Quality Thresholds:
    - Warning: interval × 2
    - Error: interval × 4
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class TopicQualityMetrics:
    """Quality metrics for a single topic."""

    topic_name: str
    expected_hz: float
    expected_interval_ms: float
    actual_mean_hz: float
    actual_mean_interval_ms: float
    message_count: int
    min_interval_ms: float
    max_interval_ms: float
    std_interval_ms: float

    warning_threshold_ms: float
    error_threshold_ms: float
    warning_count: int = 0
    error_count: int = 0

    gaps: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def warning_ratio(self) -> float:
        if self.message_count <= 1:
            return 0.0
        return self.warning_count / (self.message_count - 1)

    @property
    def error_ratio(self) -> float:
        if self.message_count <= 1:
            return 0.0
        return self.error_count / (self.message_count - 1)

    @property
    def quality_status(self) -> str:
        if self.error_count > 0:
            return "ERROR"
        if self.warning_count > 0:
            return "WARNING"
        return "GOOD"


@dataclass
class QualityReport:
    """Complete quality report for a ROSbag."""

    source_bag: str
    duration_sec: float
    total_messages: int
    topic_metrics: Dict[str, TopicQualityMetrics] = field(default_factory=dict)

    @property
    def overall_status(self) -> str:
        statuses = [m.quality_status for m in self.topic_metrics.values()]
        if "ERROR" in statuses:
            return "ERROR"
        if "WARNING" in statuses:
            return "WARNING"
        return "GOOD"

    @property
    def total_warnings(self) -> int:
        return sum(m.warning_count for m in self.topic_metrics.values())

    @property
    def total_errors(self) -> int:
        return sum(m.error_count for m in self.topic_metrics.values())

    def to_dict(self) -> Dict[str, Any]:
        return {
            "source_bag": self.source_bag,
            "duration_sec": round(self.duration_sec, 3),
            "total_messages": self.total_messages,
            "overall_status": self.overall_status,
            "summary": {
                "total_warnings": self.total_warnings,
                "total_errors": self.total_errors,
                "topics_analyzed": len(self.topic_metrics),
            },
            "topics": {
                name: {
                    "expected_hz": round(m.expected_hz, 2),
                    "actual_mean_hz": round(m.actual_mean_hz, 2),
                    "message_count": m.message_count,
                    "interval_ms": {
                        "expected": round(m.expected_interval_ms, 2),
                        "actual_mean": round(m.actual_mean_interval_ms, 2),
                        "min": round(m.min_interval_ms, 2),
                        "max": round(m.max_interval_ms, 2),
                        "std": round(m.std_interval_ms, 2),
                    },
                    "thresholds_ms": {
                        "warning": round(m.warning_threshold_ms, 2),
                        "error": round(m.error_threshold_ms, 2),
                    },
                    "quality": {
                        "status": m.quality_status,
                        "warning_count": m.warning_count,
                        "error_count": m.error_count,
                        "warning_ratio": round(m.warning_ratio * 100, 2),
                        "error_ratio": round(m.error_ratio * 100, 2),
                    },
                    "gaps": m.gaps[:20],
                }
                for name, m in self.topic_metrics.items()
            },
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    def save(self, output_path: Path):
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(self.to_json())


@dataclass
class QualityConfig:
    """Configuration for quality analysis."""

    warning_multiplier: float = 2.0
    error_multiplier: float = 4.0

    default_topic_hz: Dict[str, float] = field(
        default_factory=lambda: {
            "joint_states": 100.0,
            "joint_trajectory": 100.0,
            "image": 30.0,
            "metadata": 30.0,
        }
    )

    def get_expected_hz(self, topic_name: str, topic_type: str) -> float:
        topic_lower = topic_name.lower()
        type_lower = topic_type.lower()

        if "joint_states" in type_lower or "jointstate" in type_lower:
            return self.default_topic_hz.get("joint_states", 100.0)

        if "jointtrajectory" in type_lower or "joint_trajectory" in topic_lower:
            return self.default_topic_hz.get("joint_trajectory", 100.0)

        if "image" in type_lower or "camera" in topic_lower:
            return self.default_topic_hz.get("image", 30.0)

        if "metadata" in topic_lower:
            return self.default_topic_hz.get("metadata", 30.0)

        return 30.0


class QualityAnalyzer:
    """Analyzes ROSbag data quality by checking timestamp gaps."""

    def __init__(
        self,
        config: Optional[QualityConfig] = None,
        logger: Optional[Any] = None,
    ):
        self.config = config or QualityConfig()
        self.logger = logger

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_warning(self, msg: str):
        if self.logger:
            self.logger.warning(msg)

    def analyze_timestamps(
        self,
        topic_name: str,
        timestamps: List[float],
        expected_hz: Optional[float] = None,
        topic_type: str = "",
    ) -> TopicQualityMetrics:
        """Analyze timestamp gaps for a single topic."""
        if expected_hz is None:
            expected_hz = self.config.get_expected_hz(topic_name, topic_type)

        expected_interval_ms = 1000.0 / expected_hz
        warning_threshold_ms = expected_interval_ms * self.config.warning_multiplier
        error_threshold_ms = expected_interval_ms * self.config.error_multiplier

        if len(timestamps) < 2:
            return TopicQualityMetrics(
                topic_name=topic_name,
                expected_hz=expected_hz,
                expected_interval_ms=expected_interval_ms,
                actual_mean_hz=0.0,
                actual_mean_interval_ms=0.0,
                message_count=len(timestamps),
                min_interval_ms=0.0,
                max_interval_ms=0.0,
                std_interval_ms=0.0,
                warning_threshold_ms=warning_threshold_ms,
                error_threshold_ms=error_threshold_ms,
            )

        sorted_timestamps = sorted(timestamps)
        intervals_sec = np.diff(sorted_timestamps)
        intervals_ms = intervals_sec * 1000.0

        warning_count = 0
        error_count = 0
        gaps = []

        for i, interval_ms in enumerate(intervals_ms):
            if interval_ms > error_threshold_ms:
                error_count += 1
                gaps.append(
                    {
                        "index": i,
                        "time_sec": round(sorted_timestamps[i], 4),
                        "gap_ms": round(interval_ms, 2),
                        "severity": "error",
                        "expected_ms": round(expected_interval_ms, 2),
                    }
                )
            elif interval_ms > warning_threshold_ms:
                warning_count += 1
                gaps.append(
                    {
                        "index": i,
                        "time_sec": round(sorted_timestamps[i], 4),
                        "gap_ms": round(interval_ms, 2),
                        "severity": "warning",
                        "expected_ms": round(expected_interval_ms, 2),
                    }
                )

        duration_sec = sorted_timestamps[-1] - sorted_timestamps[0]
        actual_mean_hz = (len(timestamps) - 1) / duration_sec if duration_sec > 0 else 0

        return TopicQualityMetrics(
            topic_name=topic_name,
            expected_hz=expected_hz,
            expected_interval_ms=expected_interval_ms,
            actual_mean_hz=actual_mean_hz,
            actual_mean_interval_ms=float(np.mean(intervals_ms)),
            message_count=len(timestamps),
            min_interval_ms=float(np.min(intervals_ms)),
            max_interval_ms=float(np.max(intervals_ms)),
            std_interval_ms=float(np.std(intervals_ms)),
            warning_threshold_ms=warning_threshold_ms,
            error_threshold_ms=error_threshold_ms,
            warning_count=warning_count,
            error_count=error_count,
            gaps=gaps,
        )

    def analyze_rosbag(
        self,
        bag_path: Path,
        topic_timestamps: Dict[str, List[float]],
        topic_types: Dict[str, str],
        expected_hz_override: Optional[Dict[str, float]] = None,
    ) -> QualityReport:
        """Analyze all topics in a ROSbag."""
        expected_hz_override = expected_hz_override or {}

        topic_metrics = {}
        total_messages = 0

        all_timestamps = []
        for timestamps in topic_timestamps.values():
            all_timestamps.extend(timestamps)
            total_messages += len(timestamps)

        duration_sec = 0.0
        if all_timestamps:
            duration_sec = max(all_timestamps) - min(all_timestamps)

        for topic_name, timestamps in topic_timestamps.items():
            topic_type = topic_types.get(topic_name, "")
            expected_hz = expected_hz_override.get(topic_name)

            metrics = self.analyze_timestamps(
                topic_name=topic_name,
                timestamps=timestamps,
                expected_hz=expected_hz,
                topic_type=topic_type,
            )
            topic_metrics[topic_name] = metrics

            if metrics.quality_status != "GOOD":
                self._log_warning(
                    f"Topic {topic_name}: {metrics.quality_status} "
                    f"(warnings={metrics.warning_count}, errors={metrics.error_count}, "
                    f"max_gap={metrics.max_interval_ms:.1f}ms)"
                )

        report = QualityReport(
            source_bag=str(bag_path),
            duration_sec=duration_sec,
            total_messages=total_messages,
            topic_metrics=topic_metrics,
        )

        self._log_info(
            f"Quality analysis complete: {report.overall_status} "
            f"(warnings={report.total_warnings}, errors={report.total_errors})"
        )

        return report

    def print_summary(self, report: QualityReport):
        """Print a human-readable summary of the quality report."""
        print("\n" + "=" * 60)
        print("ROSbag Quality Report")
        print("=" * 60)
        print(f"Source: {report.source_bag}")
        print(f"Duration: {report.duration_sec:.2f}s")
        print(f"Total messages: {report.total_messages}")
        print(f"Overall status: {report.overall_status}")
        print("-" * 60)

        for name, metrics in report.topic_metrics.items():
            status_icon = {"GOOD": "✓", "WARNING": "⚠", "ERROR": "✗"}[
                metrics.quality_status
            ]
            print(f"\n{status_icon} {name}")
            print(
                f"    Expected: {metrics.expected_hz:.0f}Hz ({metrics.expected_interval_ms:.1f}ms)"
            )
            print(
                f"    Actual:   {metrics.actual_mean_hz:.1f}Hz ({metrics.actual_mean_interval_ms:.1f}ms)"
            )
            print(f"    Messages: {metrics.message_count}")
            print(f"    Max gap:  {metrics.max_interval_ms:.1f}ms")

            if metrics.warning_count > 0 or metrics.error_count > 0:
                print(
                    f"    Issues:   {metrics.warning_count} warnings, {metrics.error_count} errors"
                )

        print("\n" + "=" * 60)
