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

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import json
import numpy as np
import pyarrow.parquet as pq


class ValidationSeverity(Enum):
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"


@dataclass
class ValidationIssue:
    severity: ValidationSeverity
    category: str
    message: str
    details: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "severity": self.severity.value,
            "category": self.category,
            "message": self.message,
        }
        if self.details:
            result["details"] = self.details
        return result


@dataclass
class ValidationResult:
    is_valid: bool
    dataset_path: str
    issues: List[ValidationIssue] = field(default_factory=list)
    summary: Dict[str, Any] = field(default_factory=dict)

    @property
    def error_count(self) -> int:
        return sum(1 for i in self.issues if i.severity == ValidationSeverity.ERROR)

    @property
    def warning_count(self) -> int:
        return sum(1 for i in self.issues if i.severity == ValidationSeverity.WARNING)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "is_valid": self.is_valid,
            "dataset_path": self.dataset_path,
            "error_count": self.error_count,
            "warning_count": self.warning_count,
            "summary": self.summary,
            "issues": [i.to_dict() for i in self.issues],
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)


REQUIRED_INFO_FIELDS = {
    "codebase_version": str,
    "robot_type": str,
    "total_episodes": int,
    "total_frames": int,
    "fps": int,
    "features": dict,
    "splits": dict,
    "data_path": str,
    "chunks_size": int,
    "total_chunks": int,
    "total_tasks": int,
}

REQUIRED_PARQUET_COLUMNS = {
    "timestamp",
    "frame_index",
    "episode_index",
    "index",
    "task_index",
}

REQUIRED_FEATURE_FIELDS = {"dtype", "shape"}


class LeRobotDatasetValidator:
    def __init__(self, dataset_path: Path, logger=None):
        self.dataset_path = Path(dataset_path)
        self.logger = logger
        self._issues: List[ValidationIssue] = []
        self._info: Optional[Dict] = None
        self._episodes: List[Dict] = []
        self._episodes_stats: List[Dict] = []
        self._tasks: List[Dict] = []

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def _add_issue(
        self,
        severity: ValidationSeverity,
        category: str,
        message: str,
        details: Optional[Dict] = None,
    ):
        issue = ValidationIssue(severity, category, message, details)
        self._issues.append(issue)
        if severity == ValidationSeverity.ERROR:
            self._log_error(f"[{category}] {message}")

    def validate(self) -> ValidationResult:
        self._issues = []

        if not self.dataset_path.exists():
            self._add_issue(
                ValidationSeverity.ERROR,
                "structure",
                f"Dataset path does not exist: {self.dataset_path}",
            )
            return ValidationResult(
                is_valid=False,
                dataset_path=str(self.dataset_path),
                issues=self._issues,
            )

        self._validate_directory_structure()
        self._validate_info_json()
        self._validate_metadata_files()
        self._validate_parquet_files()
        self._validate_video_files()
        self._validate_cross_file_consistency()

        is_valid = not any(i.severity == ValidationSeverity.ERROR for i in self._issues)

        summary = self._build_summary()

        return ValidationResult(
            is_valid=is_valid,
            dataset_path=str(self.dataset_path),
            issues=self._issues,
            summary=summary,
        )

    def _validate_directory_structure(self):
        required_dirs = ["meta", "data"]
        required_files = [
            "meta/info.json",
            "meta/episodes.jsonl",
            "meta/tasks.jsonl",
        ]

        for dir_name in required_dirs:
            dir_path = self.dataset_path / dir_name
            if not dir_path.exists():
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "structure",
                    f"Required directory missing: {dir_name}/",
                )
            elif not dir_path.is_dir():
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "structure",
                    f"Expected directory but found file: {dir_name}",
                )

        for file_path in required_files:
            full_path = self.dataset_path / file_path
            if not full_path.exists():
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "structure",
                    f"Required file missing: {file_path}",
                )

        episodes_stats_path = self.dataset_path / "meta" / "episodes_stats.jsonl"
        if not episodes_stats_path.exists():
            self._add_issue(
                ValidationSeverity.WARNING,
                "structure",
                "episodes_stats.jsonl missing (required for LeRobot v2.1)",
            )

    def _validate_info_json(self):
        info_path = self.dataset_path / "meta" / "info.json"
        if not info_path.exists():
            return

        try:
            with open(info_path, "r", encoding="utf-8") as f:
                self._info = json.load(f)
        except json.JSONDecodeError as e:
            self._add_issue(
                ValidationSeverity.ERROR,
                "info.json",
                f"Invalid JSON: {e}",
            )
            return

        for field_name, expected_type in REQUIRED_INFO_FIELDS.items():
            if field_name not in self._info:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "info.json",
                    f"Required field missing: {field_name}",
                )
            elif not isinstance(self._info[field_name], expected_type):
                actual_type = type(self._info[field_name]).__name__
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "info.json",
                    f"Field '{field_name}' has wrong type: expected {expected_type.__name__}, got {actual_type}",
                )

        if self._info.get("codebase_version") not in ["v2.0", "v2.1"]:
            self._add_issue(
                ValidationSeverity.WARNING,
                "info.json",
                f"Unexpected codebase_version: {self._info.get('codebase_version')} (expected v2.0 or v2.1)",
            )

        features = self._info.get("features", {})
        for feature_name, feature_def in features.items():
            for required_field in REQUIRED_FEATURE_FIELDS:
                if required_field not in feature_def:
                    self._add_issue(
                        ValidationSeverity.ERROR,
                        "info.json",
                        f"Feature '{feature_name}' missing required field: {required_field}",
                    )

            if "shape" in feature_def:
                shape = feature_def["shape"]
                if not isinstance(shape, (list, tuple)):
                    self._add_issue(
                        ValidationSeverity.ERROR,
                        "info.json",
                        f"Feature '{feature_name}' shape must be list/tuple, got {type(shape).__name__}",
                    )
                elif any(s <= 0 for s in shape if isinstance(s, int)):
                    self._add_issue(
                        ValidationSeverity.WARNING,
                        "info.json",
                        f"Feature '{feature_name}' has non-positive shape dimension: {shape}",
                    )

    def _validate_metadata_files(self):
        self._validate_jsonl_file(
            "meta/episodes.jsonl", self._episodes, ["episode_index", "length"]
        )
        self._validate_jsonl_file(
            "meta/tasks.jsonl", self._tasks, ["task_index", "task"]
        )
        self._validate_jsonl_file(
            "meta/episodes_stats.jsonl",
            self._episodes_stats,
            ["episode_index", "stats"],
            required=False,
        )

    def _validate_jsonl_file(
        self,
        relative_path: str,
        storage: List[Dict],
        required_fields: List[str],
        required: bool = True,
    ):
        file_path = self.dataset_path / relative_path
        if not file_path.exists():
            if required:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "metadata",
                    f"Required file missing: {relative_path}",
                )
            return

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        entry = json.loads(line)
                        storage.append(entry)

                        for field in required_fields:
                            if field not in entry:
                                self._add_issue(
                                    ValidationSeverity.ERROR,
                                    "metadata",
                                    f"{relative_path} line {line_num}: missing field '{field}'",
                                )
                    except json.JSONDecodeError as e:
                        self._add_issue(
                            ValidationSeverity.ERROR,
                            "metadata",
                            f"{relative_path} line {line_num}: invalid JSON - {e}",
                        )
        except Exception as e:
            self._add_issue(
                ValidationSeverity.ERROR,
                "metadata",
                f"Failed to read {relative_path}: {e}",
            )

    def _validate_parquet_files(self):
        if not self._info:
            return

        data_dir = self.dataset_path / "data"
        if not data_dir.exists():
            return

        parquet_files = list(data_dir.glob("chunk-*/episode_*.parquet"))
        if not parquet_files:
            self._add_issue(
                ValidationSeverity.ERROR,
                "parquet",
                "No parquet files found in data/ directory",
            )
            return

        total_rows = 0
        episode_indices_found: Set[int] = set()

        for parquet_path in sorted(parquet_files):
            try:
                table = pq.read_table(parquet_path)
                df_columns = set(table.column_names)

                missing_cols = REQUIRED_PARQUET_COLUMNS - df_columns
                if missing_cols:
                    self._add_issue(
                        ValidationSeverity.ERROR,
                        "parquet",
                        f"{parquet_path.name}: missing required columns: {missing_cols}",
                    )

                features = self._info.get("features", {})
                for feature_name in features:
                    if feature_name not in df_columns and not feature_name.startswith(
                        "observation.images."
                    ):
                        self._add_issue(
                            ValidationSeverity.WARNING,
                            "parquet",
                            f"{parquet_path.name}: feature '{feature_name}' defined in info.json but not in parquet",
                        )

                num_rows = table.num_rows
                total_rows += num_rows

                if "episode_index" in df_columns:
                    ep_indices = table.column("episode_index").to_pylist()
                    unique_eps = set(ep_indices)
                    episode_indices_found.update(unique_eps)

                    if len(unique_eps) > 1:
                        self._add_issue(
                            ValidationSeverity.WARNING,
                            "parquet",
                            f"{parquet_path.name}: contains multiple episode indices: {unique_eps}",
                        )

                self._validate_parquet_data_integrity(parquet_path, table)

            except Exception as e:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "parquet",
                    f"Failed to read {parquet_path.name}: {e}",
                )

        expected_frames = self._info.get("total_frames", 0)
        if total_rows != expected_frames:
            self._add_issue(
                ValidationSeverity.ERROR,
                "parquet",
                f"Total frame count mismatch: info.json says {expected_frames}, parquet has {total_rows}",
            )

        expected_episodes = self._info.get("total_episodes", 0)
        if len(episode_indices_found) != expected_episodes:
            self._add_issue(
                ValidationSeverity.ERROR,
                "parquet",
                f"Episode count mismatch: info.json says {expected_episodes}, found {len(episode_indices_found)}",
            )

    def _validate_parquet_data_integrity(self, parquet_path: Path, table):
        columns = table.column_names

        if "observation.state" in columns:
            obs_state = table.column("observation.state")
            self._check_array_column_for_issues(
                parquet_path.name, "observation.state", obs_state
            )

        if "action" in columns:
            action = table.column("action")
            self._check_array_column_for_issues(parquet_path.name, "action", action)

        if "frame_index" in columns:
            frame_indices = table.column("frame_index").to_pylist()
            if frame_indices and frame_indices[0] != 0:
                self._add_issue(
                    ValidationSeverity.WARNING,
                    "parquet",
                    f"{parquet_path.name}: frame_index does not start at 0 (starts at {frame_indices[0]})",
                )

            for i in range(1, len(frame_indices)):
                if frame_indices[i] != frame_indices[i - 1] + 1:
                    self._add_issue(
                        ValidationSeverity.WARNING,
                        "parquet",
                        f"{parquet_path.name}: frame_index not sequential at row {i}",
                        {
                            "expected": frame_indices[i - 1] + 1,
                            "actual": frame_indices[i],
                        },
                    )
                    break

        if "timestamp" in columns:
            timestamps = table.column("timestamp").to_pylist()
            for i in range(1, len(timestamps)):
                if timestamps[i] < timestamps[i - 1]:
                    self._add_issue(
                        ValidationSeverity.ERROR,
                        "parquet",
                        f"{parquet_path.name}: timestamp not monotonically increasing at row {i}",
                        {"prev": timestamps[i - 1], "curr": timestamps[i]},
                    )
                    break

    def _check_array_column_for_issues(self, filename: str, col_name: str, column):
        try:
            data = column.to_pandas()

            empty_count = 0
            nan_count = 0
            inf_count = 0
            all_zero_count = 0

            for idx, row in enumerate(data):
                if row is None:
                    empty_count += 1
                    continue

                arr = np.array(row, dtype=np.float32)

                if len(arr) == 0:
                    empty_count += 1
                elif np.isnan(arr).any():
                    nan_count += 1
                elif np.isinf(arr).any():
                    inf_count += 1
                elif np.allclose(arr, 0):
                    all_zero_count += 1

            if empty_count > 0:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "data_integrity",
                    f"{filename}: '{col_name}' has {empty_count} empty/null rows",
                )

            if nan_count > 0:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "data_integrity",
                    f"{filename}: '{col_name}' has {nan_count} rows with NaN values",
                )

            if inf_count > 0:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "data_integrity",
                    f"{filename}: '{col_name}' has {inf_count} rows with Inf values",
                )

            total_rows = len(data)
            if all_zero_count > 0 and all_zero_count == total_rows:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "data_integrity",
                    f"{filename}: '{col_name}' has ALL rows as zeros ({all_zero_count}/{total_rows})",
                )
            elif all_zero_count > total_rows * 0.5:
                self._add_issue(
                    ValidationSeverity.WARNING,
                    "data_integrity",
                    f"{filename}: '{col_name}' has {all_zero_count}/{total_rows} ({all_zero_count * 100 // total_rows}%) rows as all zeros",
                )

        except Exception as e:
            self._add_issue(
                ValidationSeverity.WARNING,
                "data_integrity",
                f"{filename}: Failed to check '{col_name}' for issues: {e}",
            )

    def _validate_video_files(self):
        if not self._info:
            return

        video_path_pattern = self._info.get("video_path")
        if not video_path_pattern:
            return

        features = self._info.get("features", {})
        video_features = [f for f in features if f.startswith("observation.images.")]

        if not video_features:
            return

        videos_dir = self.dataset_path / "videos"
        if not videos_dir.exists():
            self._add_issue(
                ValidationSeverity.ERROR,
                "video",
                "videos/ directory missing but video features defined in info.json",
            )
            return

        for episode in self._episodes:
            ep_idx = episode.get("episode_index", 0)
            expected_length = episode.get("length", 0)
            chunk_idx = ep_idx // self._info.get("chunks_size", 1000)

            for video_feature in video_features:
                video_path = (
                    videos_dir
                    / f"chunk-{chunk_idx:03d}"
                    / video_feature
                    / f"episode_{ep_idx:06d}.mp4"
                )

                if not video_path.exists():
                    self._add_issue(
                        ValidationSeverity.ERROR,
                        "video",
                        f"Video file missing: {video_path.relative_to(self.dataset_path)}",
                    )
                    continue

                self._validate_video_frame_count(video_path, expected_length, ep_idx)

    def _validate_video_frame_count(
        self, video_path: Path, expected_frames: int, episode_idx: int
    ):
        try:
            import cv2

            cap = cv2.VideoCapture(str(video_path))
            if not cap.isOpened():
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "video",
                    f"Cannot open video: {video_path.name}",
                )
                return

            actual_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cap.release()

            if actual_frames != expected_frames:
                diff = abs(actual_frames - expected_frames)
                diff_pct = (
                    (diff / expected_frames * 100) if expected_frames > 0 else 100
                )

                severity = (
                    ValidationSeverity.ERROR
                    if diff_pct > 5
                    else ValidationSeverity.WARNING
                )

                self._add_issue(
                    severity,
                    "video",
                    f"Episode {episode_idx}: video frame count mismatch",
                    {
                        "video": video_path.name,
                        "expected": expected_frames,
                        "actual": actual_frames,
                        "diff_percent": round(diff_pct, 2),
                    },
                )

        except ImportError:
            self._add_issue(
                ValidationSeverity.WARNING,
                "video",
                "OpenCV not available, skipping video frame count validation",
            )
        except Exception as e:
            self._add_issue(
                ValidationSeverity.WARNING,
                "video",
                f"Failed to validate video {video_path.name}: {e}",
            )

    def _validate_cross_file_consistency(self):
        if not self._info:
            return

        info_episodes = self._info.get("total_episodes", 0)
        jsonl_episodes = len(self._episodes)

        if info_episodes != jsonl_episodes:
            self._add_issue(
                ValidationSeverity.ERROR,
                "consistency",
                f"Episode count mismatch: info.json={info_episodes}, episodes.jsonl={jsonl_episodes}",
            )

        info_tasks = self._info.get("total_tasks", 0)
        jsonl_tasks = len(self._tasks)

        if info_tasks != jsonl_tasks:
            self._add_issue(
                ValidationSeverity.ERROR,
                "consistency",
                f"Task count mismatch: info.json={info_tasks}, tasks.jsonl={jsonl_tasks}",
            )

        for episode in self._episodes:
            ep_idx = episode.get("episode_index")
            length = episode.get("length", 0)

            if length <= 0:
                self._add_issue(
                    ValidationSeverity.ERROR,
                    "consistency",
                    f"Episode {ep_idx} has invalid length: {length}",
                )

        episode_indices = [e.get("episode_index") for e in self._episodes]
        expected_indices = list(range(len(self._episodes)))

        if sorted(episode_indices) != expected_indices:
            self._add_issue(
                ValidationSeverity.WARNING,
                "consistency",
                f"Episode indices are not sequential 0 to N-1: {episode_indices}",
            )

    def _build_summary(self) -> Dict[str, Any]:
        summary: Dict[str, Any] = {
            "dataset_path": str(self.dataset_path),
            "error_count": sum(
                1 for i in self._issues if i.severity == ValidationSeverity.ERROR
            ),
            "warning_count": sum(
                1 for i in self._issues if i.severity == ValidationSeverity.WARNING
            ),
        }

        if self._info:
            summary["codebase_version"] = self._info.get("codebase_version")
            summary["robot_type"] = self._info.get("robot_type")
            summary["total_episodes"] = self._info.get("total_episodes")
            summary["total_frames"] = self._info.get("total_frames")
            summary["fps"] = self._info.get("fps")
            summary["features"] = list(self._info.get("features", {}).keys())

        return summary


def validate_lerobot_dataset(dataset_path: str, logger=None) -> ValidationResult:
    validator = LeRobotDatasetValidator(Path(dataset_path), logger)
    return validator.validate()
