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
# Author: Dongyun Kim, Seongwoo Kim

import json
import os
from pathlib import Path
import queue
import re
import shutil
import socket
import threading
import time
import xml.etree.ElementTree as ET

from huggingface_hub import (
    DatasetCard,
    DatasetCardData,
    HfApi,
    ModelCard,
    ModelCardData,
)
from physical_ai_interfaces.msg import TaskInfo, TaskStatus
from physical_ai_server.data_processing.data_converter import DataConverter
from physical_ai_server.data_processing.progress_tracker import (
    HuggingFaceProgressTqdm
)
from physical_ai_server.device_manager.cpu_checker import CPUChecker
from physical_ai_server.device_manager.ram_checker import RAMChecker
from physical_ai_server.device_manager.storage_checker import StorageChecker


class DataManager:

    # Progress queue for multiprocessing communication
    _progress_queue = None

    ROSBAG_ROOT = '/workspace/rosbag2'

    # Canonical ordering for primitive descriptions; index of a primitive
    # in this list is written to episode_info.json as `primitive_index`.
    # Keep in sync with
    # physical_ai_manager/src/constants/primitiveDescriptions.js.
    PRIMITIVE_DESCRIPTIONS = (
        'move_to',
        'pick_up',
        'place',
        'push_pull',
        'open',
        'close',
        'button_switch',
        'pour_dispense',
        'tool_use',
        'handover_attach',
    )

    # Camera frame rate — frame_duration fields in episode_info.json are in
    # units of camera frames, so segment timing is always converted using
    # this constant regardless of control loop frequency.
    CAMERA_FPS = 15

    def __init__(
            self,
            save_root_path,
            robot_type,
            task_info):
        self._robot_type = robot_type
        self._save_path = save_root_path
        self._task_info = task_info

        # Archive path: /workspace/rosbag2/{robot_type}_{task_name}
        task_name = (getattr(task_info, 'task_name', '') or '').strip()
        self._save_repo_name = f'{robot_type}_{task_name}'
        self._save_rosbag_path = os.path.join(
            self.ROSBAG_ROOT, self._save_repo_name)

        self._record_episode_count = self._find_next_episode_number_in(
            self._save_rosbag_path)
        self._start_time_s = 0
        self._proceed_time = 0
        self._status = 'idle'  # idle | seg_recording | between_segments
        self._cpu_checker = CPUChecker()
        self.data_converter = DataConverter()
        self.current_instruction = (
            task_info.task_instruction[0]
            if task_info.task_instruction else '')
        self._init_task_limits()
        self._current_scenario_number = 0
        self._single_task = len(task_info.task_instruction) <= 1

        # Segmented episode state
        self._current_segment_index = 0
        self._segments_meta = []
        self._pending_primitive = ''
        self._segment_start_time_s = 0

    @staticmethod
    def _find_next_episode_number_in(rosbag_dir) -> int:
        """Return the smallest unused numeric episode subfolder index."""
        if not os.path.exists(rosbag_dir):
            return 0
        existing = []
        try:
            for item in os.listdir(rosbag_dir):
                path = os.path.join(rosbag_dir, item)
                if os.path.isdir(path) and item.isdigit():
                    existing.append(int(item))
        except OSError as e:
            print(f'[DataManager] Error scanning {rosbag_dir}: {e}')
            return 0
        return (max(existing) + 1) if existing else 0

    @classmethod
    def _primitive_index_for(cls, description):
        """Return the canonical index of a primitive description, or -1."""
        try:
            return cls.PRIMITIVE_DESCRIPTIONS.index(description)
        except ValueError:
            return -1

    def _current_fps(self):
        """Camera fps used for converting segment durations to frame counts."""
        return self.CAMERA_FPS

    def _serialize_segments(self):
        """Convert internal segments_meta (with frame_count) to JSON shape.

        Each output entry has exactly three fields:
            - primitive_index        (int)
            - primitive_description  (str)
            - frame_duration         ([start_frame, end_frame]) cumulative.
        """
        out = []
        cur = 0
        for seg in self._segments_meta:
            fc = int(seg.get('frame_count', 0))
            out.append({
                'primitive_index': int(seg.get(
                    'primitive_index',
                    self._primitive_index_for(
                        seg.get('primitive_description', '')))),
                'primitive_description':
                    seg.get('primitive_description', ''),
                'frame_duration': [cur, cur + fc],
            })
            cur += fc
        return out

    # (restore_pending / _prune_orphan_segments removed — no scratch flow)

    def get_status(self):
        return self._status

    # ========== Simplified Recording Methods (rosbag2-only mode) ==========

    # ========== Segment-based recording (v2) ==========

    def start_segment(self, primitive_description: str = ''):
        """Begin a new segment within the current episode."""
        if self._status not in ('idle', 'between_segments'):
            raise RuntimeError(
                f'Cannot start segment in status {self._status}')
        if self._status == 'idle':
            self._segments_meta = []
            self._current_segment_index = 0
        self._pending_primitive = primitive_description or ''
        self._status = 'seg_recording'
        self._segment_start_time_s = time.perf_counter()
        self._start_time_s = self._segment_start_time_s
        if self._task_info and getattr(self._task_info, 'task_instruction', None):
            self.current_instruction = self._task_info.task_instruction[0]
        print(f'[DataManager] Segment {self._current_segment_index} started '
              f'(primitive={primitive_description!r})')

    def stop_segment(self):
        """End the current segment and record its metadata."""
        if self._status != 'seg_recording':
            raise RuntimeError(
                f'Cannot stop segment in status {self._status}')
        now = time.perf_counter()
        duration = now - self._segment_start_time_s \
            if self._segment_start_time_s else 0.0
        fps = self._current_fps()
        frame_count = max(0, int(round(duration * fps)))
        seg = {
            'primitive_index': self._primitive_index_for(
                self._pending_primitive),
            'primitive_description': self._pending_primitive,
            'frame_count': frame_count,
        }
        self._segments_meta.append(seg)
        self._current_segment_index += 1
        self._status = 'between_segments'
        self._start_time_s = 0
        self._segment_start_time_s = 0
        self._pending_primitive = ''
        print(f'[DataManager] Segment {len(self._segments_meta) - 1} stopped '
              f'({duration:.2f}s, {frame_count} frames @ {fps}Hz)')

    def discard_segment(self, idx: int):
        """Remove segment `idx` from the current (in-progress) episode dir
        and renumber trailing segments."""
        if self._status != 'between_segments':
            raise RuntimeError(
                f'Cannot discard segment in status {self._status}')
        if not (0 <= idx < len(self._segments_meta)):
            raise ValueError(f'Invalid segment index {idx}')
        seg_root = os.path.join(self.get_episode_dir(), 'segments')
        target = os.path.join(seg_root, str(idx))
        if os.path.isdir(target):
            shutil.rmtree(target)
        for i in range(idx + 1, len(self._segments_meta)):
            src = os.path.join(seg_root, str(i))
            dst = os.path.join(seg_root, str(i - 1))
            if os.path.isdir(src):
                os.rename(src, dst)
        self._segments_meta.pop(idx)
        self._current_segment_index = len(self._segments_meta)
        if not self._segments_meta:
            self._status = 'idle'
            # Nothing left — remove the empty episode dir so we don't leave
            # an orphaned slot.
            ep_dir = self.get_episode_dir()
            try:
                if os.path.isdir(ep_dir):
                    shutil.rmtree(ep_dir)
            except OSError:
                pass
        print(f'[DataManager] Segment {idx} discarded. '
              f'Remaining: {len(self._segments_meta)}')

    def discard_episode(self):
        """Throw away all segments of the in-progress episode."""
        if self._status not in ('idle', 'between_segments'):
            raise RuntimeError(
                f'Cannot discard episode in status {self._status}')
        ep_dir = self._episode_dir(self._record_episode_count)
        if os.path.exists(ep_dir):
            shutil.rmtree(ep_dir, ignore_errors=True)
        self._segments_meta = []
        self._current_segment_index = 0
        self._status = 'idle'
        print('[DataManager] Episode discarded')

    def finish_episode(self, urdf_path: str = None):
        """Finalise the in-progress episode: consolidate mcaps, write
        ``metadata.yaml`` + ``episode_info.json`` + ``robot.urdf``, and
        advance the episode counter.

        Segments were recorded directly under ``{ep_dir}/segments/{idx}/``;
        this method copies each segment's mcap to ``{ep_idx}_{seg}.mcap``
        in the episode root, writes the unified ``metadata.yaml``, and
        removes the temporary ``segments/`` subdir.
        """
        if self._status != 'between_segments':
            raise RuntimeError(
                f'Cannot finish episode in status {self._status}')
        if not self._segments_meta:
            raise RuntimeError('No segments recorded for this episode')

        from physical_ai_server.data_processing.mcap_merger import (
            archive_segments,
        )

        ep_idx = self._record_episode_count
        ep_dir = self._episode_dir(ep_idx)
        seg_root = os.path.join(ep_dir, 'segments')
        seg_names = sorted(
            (d for d in os.listdir(seg_root) if d.isdigit()),
            key=int,
        )
        seg_paths = [os.path.join(seg_root, d) for d in seg_names]

        try:
            archive_segments(seg_paths, ep_dir, ep_idx=ep_idx)
        except Exception as e:
            raise RuntimeError(f'archive_segments failed: {e}') from e

        if urdf_path and os.path.exists(urdf_path):
            urdf_dest = os.path.join(ep_dir, 'robot.urdf')
            try:
                self._write_urdf_stripped(urdf_path, urdf_dest)
            except Exception as e:
                print(f'[DataManager] URDF copy failed: {e}')

        try:
            self._write_episode_info_v1(ep_dir, self._task_info, ep_idx)
        except Exception as e:
            print(f'[DataManager] Failed to write episode_info: {e}')

        # Clean up the segments/ working area.
        shutil.rmtree(seg_root, ignore_errors=True)

        self._segments_meta = []
        self._current_segment_index = 0
        self._record_episode_count = ep_idx + 1
        self._status = 'idle'
        print(f'[DataManager] Episode {ep_idx} finished -> {ep_dir}')
        return Path(ep_dir)

    @staticmethod
    def _write_urdf_stripped(src, dst):
        """Copy a URDF file while dropping XML comments and blank-only lines.

        xacro-generated URDFs include verbose banner comments that add noise
        to the archived episode; strip them for cleaner storage.
        """
        with open(src, 'r', encoding='utf-8') as f:
            text = f.read()
        text = re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)
        text = re.sub(r'\n\s*\n+', '\n', text)
        with open(dst, 'w', encoding='utf-8') as f:
            f.write(text.strip() + '\n')

    def _write_episode_info_v1(self, archive_dir, task_info, episode_index):
        """Write a v1-compatible episode_info.json with segment extensions."""
        task_num = (getattr(task_info, 'task_num', '') or '')
        task_name = (getattr(task_info, 'task_name', '') or '')
        instr_list = getattr(task_info, 'task_instruction', []) or []
        task_instruction = instr_list[0] if instr_list else ''
        fps = self.CAMERA_FPS

        meta = {
            'task_instruction': task_instruction,
            'robot_type': self._robot_type,
            'episode_index': episode_index,
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'fps': fps,
            'format_version': 'robotis_v1',
            'device_serial': socket.gethostname(),
            'needs_review': False,
            # --- segment extensions (ignored by legacy tooling) ---
            'task_num': task_num,
            'task_name': task_name,
            'segments': self._serialize_segments(),
        }
        info_path = os.path.join(archive_dir, 'episode_info.json')
        with open(info_path, 'w') as f:
            json.dump(meta, f, indent=2)
        print(f'[ROBOTIS] episode_info.json written: {info_path}')

    # ========== Legacy wrappers (single-segment semantics) ==========

    def start_recording(self):
        """Legacy: start a single segment using task_info.primitive_description."""
        primitive = ''
        if self._task_info is not None:
            primitive = getattr(
                self._task_info, 'primitive_description', '') or ''
        self.start_segment(primitive)

    def stop_recording(self):
        """Legacy: stop the current segment; no auto-finalize under scratch flow."""
        if self._status == 'seg_recording':
            self.stop_segment()
        self._start_time_s = 0

    def is_recording(self):
        """Check if currently recording a segment."""
        return self._status == 'seg_recording'

    # ========== Paths ==========

    def _episode_dir(self, ep_idx):
        return os.path.join(self._save_rosbag_path, str(ep_idx))

    def get_episode_dir(self):
        """Absolute path of the in-progress episode directory."""
        return self._episode_dir(self._record_episode_count)

    def get_save_rosbag_path(self, allow_idle: bool = False):
        """Absolute path of the current segment's rosbag directory.

        Segments are recorded directly under the final episode dir:
            {archive}/{ep_idx}/segments/{seg_idx}
        """
        if self._status == 'idle' and not allow_idle:
            return None
        return os.path.join(
            self.get_episode_dir(),
            'segments',
            str(self._current_segment_index),
        )

    def save_robotis_metadata(self, urdf_path: str = None, needs_review: bool = False):
        """No-op under the direct-record flow (metadata is written on
        finish_episode). Kept for backward compatibility with legacy
        callers."""
        return

    def toggle_previous_episode_needs_review(self):
        """Toggle the previous episode's needs_review flag.

        Reads the current needs_review state and flips it.
        Used when user presses cancel in idle state.

        Returns:
            Optional[bool]: The new needs_review value, or None if failed.
        """
        prev_episode = self._record_episode_count - 1
        if prev_episode < 0:
            print('[DataManager] No previous episode to toggle')
            return None

        episode_path = self._save_rosbag_path + f'/{prev_episode}'
        meta_data_path = os.path.join(episode_path, 'episode_info.json')

        if not os.path.exists(meta_data_path):
            # Backward compatibility: check old name
            legacy_path = os.path.join(episode_path, 'meta_data.json')
            if os.path.exists(legacy_path):
                meta_data_path = legacy_path
            else:
                print(f'[DataManager] No episode_info.json at: {meta_data_path}')
                return None

        try:
            with open(meta_data_path, 'r') as f:
                meta_data = json.load(f)

            current = meta_data.get('needs_review', False)
            meta_data['needs_review'] = not current

            with open(meta_data_path, 'w') as f:
                json.dump(meta_data, f, indent=2)

            print(f'[DataManager] Episode {prev_episode} '
                  f'needs_review: {current} -> {not current}')
            return not current
        except Exception as e:
            print(f'[DataManager] Failed to toggle episode {prev_episode}: {e}')
            return None

    def _copy_urdf_with_meshes(
        self,
        urdf_path: str,
        urdf_dest: str,
        output_dir: str
    ):
        """
        Copy URDF file and all referenced mesh files.

        Parses URDF to find mesh references (package:// or file://),
        copies them to meshes/ subdirectory, and updates URDF paths.

        Args:
            urdf_path: Source URDF file path.
            urdf_dest: Destination URDF file path.
            output_dir: Output directory for meshes.
        """
        # Parse URDF
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Find all mesh elements
        mesh_elements = root.findall('.//mesh')
        if not mesh_elements:
            # No meshes, just copy URDF
            shutil.copy2(urdf_path, urdf_dest)
            return

        # Create meshes directory
        meshes_dir = os.path.join(output_dir, 'meshes')
        os.makedirs(meshes_dir, exist_ok=True)

        # Track copied meshes to avoid duplicates
        copied_meshes = {}
        mesh_count = 0

        # Known ROS package paths to search
        ros_pkg_paths = [
            '/root/main_ws/ai_worker',
            '/root/ros2_ws/install',
            '/root/ros2_ws/src',
            '/opt/ros/humble/share',
        ]

        for mesh_elem in mesh_elements:
            filename = mesh_elem.get('filename')
            if not filename:
                continue

            # Resolve the actual file path
            actual_path = self._resolve_mesh_path(filename, ros_pkg_paths)
            if actual_path is None or not os.path.exists(actual_path):
                print(f'[ROBOTIS] Warning: Mesh not found: {filename}')
                continue

            # Check if already copied
            if actual_path in copied_meshes:
                mesh_elem.set('filename', copied_meshes[actual_path])
                continue

            # Determine relative path in meshes directory
            # Extract package structure: package://pkg_name/meshes/... -> meshes/pkg_name/...
            rel_path = self._get_mesh_relative_path(filename, actual_path)
            dest_mesh_path = os.path.join(meshes_dir, rel_path)

            # Create subdirectories
            os.makedirs(os.path.dirname(dest_mesh_path), exist_ok=True)

            # Copy mesh file
            try:
                shutil.copy2(actual_path, dest_mesh_path)
                mesh_count += 1
            except Exception as e:
                print(f'[ROBOTIS] Warning: Failed to copy mesh {actual_path}: {e}')
                continue

            # Update URDF reference to relative path
            new_filename = f'meshes/{rel_path}'
            mesh_elem.set('filename', new_filename)
            copied_meshes[actual_path] = new_filename

        # Write modified URDF
        tree.write(urdf_dest, encoding='unicode', xml_declaration=True)
        print(f'[ROBOTIS] Copied {mesh_count} mesh files')

    def _resolve_mesh_path(self, filename: str, search_paths: list) -> str:
        """
        Resolve mesh file path from package:// or file:// URI.

        Args:
            filename: Mesh filename (package:// or file:// or absolute path).
            search_paths: List of paths to search for packages.

        Returns:
            Absolute file path or None if not found.
        """
        # Handle file:// prefix
        if filename.startswith('file://'):
            return filename[7:]

        # Handle absolute paths
        if filename.startswith('/'):
            return filename

        # Handle package:// prefix
        if filename.startswith('package://'):
            # Extract package name and relative path
            # package://ffw_description/meshes/... -> ffw_description, meshes/...
            path_part = filename[10:]  # Remove 'package://'
            parts = path_part.split('/', 1)
            if len(parts) != 2:
                return None

            pkg_name, rel_path = parts

            # Search for package in known paths
            for search_path in search_paths:
                # Try direct path: search_path/pkg_name/rel_path
                candidate = os.path.join(search_path, pkg_name, rel_path)
                if os.path.exists(candidate):
                    return candidate

                # Try share path: search_path/pkg_name/share/pkg_name/rel_path
                candidate = os.path.join(
                    search_path, pkg_name, 'share', pkg_name, rel_path
                )
                if os.path.exists(candidate):
                    return candidate

        return None

    def _get_mesh_relative_path(self, filename: str, actual_path: str) -> str:
        """
        Get relative path for mesh file in output directory.

        Args:
            filename: Original mesh filename reference.
            actual_path: Resolved absolute path.

        Returns:
            Relative path to use in output meshes directory.
        """
        # For package:// URIs, use package structure
        if filename.startswith('package://'):
            # package://ffw_description/meshes/common/... -> ffw_description/common/...
            path_part = filename[10:]
            parts = path_part.split('/', 2)
            if len(parts) >= 3 and parts[1] == 'meshes':
                # Skip the 'meshes' part: pkg_name/subpath
                return f'{parts[0]}/{parts[2]}'
            elif len(parts) >= 2:
                return '/'.join(parts)

        # For other cases, use just the filename
        return os.path.basename(actual_path)

    def should_record_rosbag2(self):
        """In simplified mode, always record rosbag2."""
        # Always return True in rosbag2-only mode
        # Legacy: return self._task_info.record_rosbag2
        return True

    def get_current_record_status(self):
        current_status = TaskStatus()
        current_status.robot_type = self._robot_type
        current_status.task_info = self._task_info or TaskInfo()

        # Simplified mode statuses
        if self._status == 'idle':
            current_status.phase = TaskStatus.READY
            current_status.total_time = int(0)
        elif self._status == 'seg_recording' or self._status == 'recording':
            current_status.phase = TaskStatus.RECORDING
            # Calculate elapsed time
            if self._start_time_s > 0:
                elapsed = time.perf_counter() - self._start_time_s
                self._proceed_time = int(elapsed)
            current_status.total_time = int(0)  # No time limit in simplified mode
        elif self._status == 'between_segments':
            current_status.phase = TaskStatus.STOPPED
            current_status.total_time = int(0)
            self._proceed_time = 0
        elif self._status == 'merging':
            current_status.phase = TaskStatus.CONVERTING
            current_status.total_time = int(0)
            self._proceed_time = 0
        # Legacy statuses (for backward compatibility)
        elif self._status == 'warmup':
            current_status.phase = TaskStatus.WARMING_UP
            current_status.total_time = int(self._task_info.warmup_time_s)
        elif self._status == 'run':
            current_status.phase = TaskStatus.RECORDING
            current_status.total_time = int(self._task_info.episode_time_s)
        elif self._status == 'reset':
            current_status.phase = TaskStatus.RESETTING
            current_status.total_time = int(self._task_info.reset_time_s)
        elif self._status == 'save' or self._status == 'finish':
            is_saving, encoding_progress = self._get_encoding_progress()
            current_status.phase = TaskStatus.SAVING
            current_status.total_time = int(0)
            self._proceed_time = int(0)
            if is_saving:
                current_status.encoding_progress = encoding_progress
            else:
                current_status.encoding_progress = 0.0
        elif self._status == 'stop':
            is_saving, encoding_progress = self._get_encoding_progress()
            current_status.total_time = int(0)
            self._proceed_time = int(0)
            if is_saving:
                current_status.phase = TaskStatus.SAVING
                current_status.encoding_progress = encoding_progress
            else:
                current_status.phase = TaskStatus.STOPPED

        current_status.current_task_instruction = self.current_instruction
        current_status.proceed_time = int(getattr(self, '_proceed_time', 0))
        current_status.current_episode_number = int(self._record_episode_count)
        current_status.current_segment_index = int(self._current_segment_index)
        current_status.segment_count = int(len(self._segments_meta))
        current_status.segment_primitives = [
            s.get('primitive_description', '') for s in self._segments_meta
        ]
        current_status.merge_status = 'none'

        total_storage, used_storage = StorageChecker.get_storage_gb('/')
        current_status.used_storage_size = float(used_storage)
        current_status.total_storage_size = float(total_storage)

        current_status.used_cpu = float(self._cpu_checker.get_cpu_usage())

        ram_total, ram_used = RAMChecker.get_ram_gb()
        current_status.used_ram_size = float(ram_used)
        current_status.total_ram_size = float(ram_total)
        if not self._single_task:
            current_status.current_scenario_number = self._current_scenario_number

        return current_status

    def _get_encoding_progress(self):
        """Get encoding progress. Always returns not-saving for rosbag2-only mode."""
        return False, 100.0

    def _init_task_limits(self):
        if not self._single_task:
            if hasattr(self._task_info, 'num_episodes'):
                self._task_info.num_episodes = 1_000_000
            if hasattr(self._task_info, 'episode_time_s'):
                self._task_info.episode_time_s = 1_000_000

    @staticmethod
    def get_robot_type_from_info_json(info_json_path):
        with open(info_json_path, 'r', encoding='utf-8') as f:
            info = json.load(f)
        return info.get('robot_type', '')

    @staticmethod
    def whoami_huggingface(endpoint, token, timeout_s=5.0):
        """Validate ``token`` against ``endpoint`` and return the user's
        identifier list (primary user + every org they belong to).

        Returns ``None`` on timeout, raises on invalid token / network error.
        Both ``endpoint`` and ``token`` are required — there is no global
        fallback.
        """
        if not endpoint:
            raise ValueError('endpoint is required')
        if not token:
            raise ValueError('token is required')

        def api_call():
            api = HfApi(endpoint=endpoint, token=token)
            user_info = api.whoami()
            user_ids = [user_info['name']]
            for org_info in user_info.get('orgs', []) or []:
                org_name = org_info.get('name')
                if org_name:
                    user_ids.append(org_name)
            return user_ids

        result_queue = queue.Queue()

        def worker():
            try:
                result_queue.put(('success', api_call()))
            except Exception as e:
                result_queue.put(('error', e))

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

        try:
            status, data = result_queue.get(timeout=timeout_s)
        except queue.Empty:
            print(f'Token validation timed out after {timeout_s}s '
                  f'for endpoint {endpoint}')
            return None

        if status == 'success':
            return data
        raise data

    # Default download roots used when the caller does not pass an explicit
    # ``local_dir``. The previous LeRobot defaults are kept as a fallback for
    # back-compat, but new flows should always pass the destination from the
    # UI so the user can pick where downloads land.
    DEFAULT_DOWNLOAD_PATHS = {
        'dataset': Path('/workspace/rosbag2'),
        'model': Path(
            '/root/ros2_ws/src/physical_ai_tools/third_party/groot/workspace/checkpoints'
        ),
    }

    @staticmethod
    def download_huggingface_repo(
        repo_id,
        repo_type='dataset',
        local_dir=None,
        endpoint=None,
        token=None,
    ):
        """Download a HuggingFace repo via the ``hf`` CLI in a PTY.

        We shell out to the CLI (instead of the huggingface_hub Python API)
        because the in-process tqdm wrapper fails to track byte counts when
        the hf-xet accelerator is used; running the CLI under a real PTY
        gives us native tqdm bars on the backend log.
        """
        import ctypes
        import os
        import pty
        import re
        import select
        import signal
        import subprocess

        if local_dir:
            save_dir = Path(local_dir) / repo_id
        else:
            base = DataManager.DEFAULT_DOWNLOAD_PATHS.get(repo_type)
            if base is None:
                raise ValueError(f'Invalid repo type: {repo_type}')
            save_dir = base / repo_id
        save_dir.mkdir(parents=True, exist_ok=True)

        env = os.environ.copy()
        if token:
            env['HF_TOKEN'] = token
        if endpoint:
            env['HF_ENDPOINT'] = endpoint
        env.pop('HF_HUB_DISABLE_PROGRESS_BARS', None)

        cmd = [
            'hf', 'download', repo_id,
            '--repo-type', repo_type,
            '--local-dir', str(save_dir),
        ]
        print(
            f'Starting download of {repo_id} ({repo_type}) from '
            f'{endpoint or "<default endpoint>"} via hf CLI'
        )

        # Child becomes a process-group leader + dies if our worker dies.
        try:
            _libc = ctypes.CDLL('libc.so.6', use_errno=True)
        except OSError:
            _libc = None

        def _preexec():
            os.setsid()
            if _libc is not None:
                try:
                    _libc.prctl(1, signal.SIGTERM, 0, 0, 0)  # PR_SET_PDEATHSIG
                except Exception:
                    pass

        master_fd, slave_fd = pty.openpty()
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=slave_fd,
                stderr=slave_fd,
                stdin=subprocess.DEVNULL,
                env=env,
                preexec_fn=_preexec,
                close_fds=True,
            )
        finally:
            os.close(slave_fd)

        ansi_re = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
        buf = b''
        try:
            while True:
                if proc.poll() is not None:
                    try:
                        data = os.read(master_fd, 8192)
                    except OSError:
                        data = b''
                    if data:
                        buf += data
                    if not data:
                        break

                try:
                    r, _, _ = select.select([master_fd], [], [], 0.5)
                except (OSError, ValueError):
                    break
                if r:
                    try:
                        data = os.read(master_fd, 8192)
                    except OSError:
                        break
                    if not data:
                        break
                    buf += data

                # Split on CR or LF: tqdm uses CR for in-place updates.
                while True:
                    idx = -1
                    for sep in (b'\r', b'\n'):
                        i = buf.find(sep)
                        if i >= 0 and (idx == -1 or i < idx):
                            idx = i
                    if idx < 0:
                        break
                    raw, buf = buf[:idx], buf[idx + 1:]
                    line = ansi_re.sub('', raw.decode('utf-8', errors='replace')).strip()
                    if not line:
                        continue
                    print(f'[hf cli] {line}')
        finally:
            try:
                os.close(master_fd)
            except OSError:
                pass

        return_code = proc.wait()
        if return_code == 0:
            print(f'Download completed: {repo_id}')
            return str(save_dir)

        print(f'Error downloading HuggingFace repo (exit={return_code}): {repo_id}')
        return False

    @classmethod
    def set_progress_queue(cls, progress_queue):
        """Set progress queue for multiprocessing communication."""
        cls._progress_queue = progress_queue

    @staticmethod
    def _create_dataset_card(local_dir, readme_path):
        """
        Create DatasetCard README for dataset repository.

        Args:
        ----
        local_dir: Local directory path containing dataset
        readme_path: Path where README.md will be saved

        """
        # Load meta/info.json for dataset structure info
        info_path = Path(local_dir) / 'meta' / 'info.json'
        dataset_info = None
        if info_path.exists():
            with open(info_path, 'r', encoding='utf-8') as f:
                dataset_info = json.load(f)

        # Prepare tags
        tags = ['robotis', 'LeRobot']
        robot_type = DataManager.get_robot_type_from_info_json(info_path)
        if robot_type and robot_type != '':
            tags.append(robot_type)

        # Create DatasetCardData
        card_data = DatasetCardData(
            license='apache-2.0',
            tags=tags,
            task_categories=['robotics'],
            configs=[
                {
                    'config_name': 'default',
                    'data_files': 'data/*/*.parquet',
                }
            ],
        )

        # Prepare dataset structure section
        dataset_structure = ''
        if dataset_info:
            dataset_structure = '[meta/info.json](meta/info.json):\n'
            dataset_structure += '```json\n'
            info_json = json.dumps(dataset_info, indent=4)
            dataset_structure += f'{info_json}\n'
            dataset_structure += '```\n'

        # Get template path
        template_dir = Path(__file__).parent
        template_path = str(template_dir / 'dataset_card_template.md')

        # Create card from template
        card = DatasetCard.from_template(
            card_data,
            template_path=template_path,
            dataset_structure=dataset_structure,
            license='apache-2.0',
        )
        card.save(str(readme_path))
        print('Dataset README.md created using HuggingFace Hub')

    @staticmethod
    def _create_model_card(local_dir, readme_path):
        """
        Create ModelCard README for model repository.

        Args:
        ----
        local_dir: Local directory path containing model
        readme_path: Path where README.md will be saved

        """
        # Find train_config.json (check common locations first)
        train_config = None
        common_paths = [
            Path(local_dir) / 'train_config.json',
            Path(local_dir) / 'config' / 'train_config.json',
            Path(local_dir) / 'pretrained_model' / 'train_config.json',
        ]

        # Check common paths first (fast)
        for config_path in common_paths:
            if config_path.exists():
                try:
                    with open(config_path, 'r', encoding='utf-8') as f:
                        train_config = json.load(f)
                    print(f'Found train_config.json at {config_path}')
                    break
                except Exception as e:
                    print(f'Error reading {config_path}: {e}')
                    continue

        # If not found, search recursively (slower fallback)
        if train_config is None:
            for config_path in Path(local_dir).rglob('train_config.json'):
                try:
                    with open(config_path, 'r', encoding='utf-8') as f:
                        train_config = json.load(f)
                    print(f'Found train_config.json at {config_path}')
                    break
                except Exception as e:
                    print(f'Error reading {config_path}: {e}')
                    continue

        if train_config is None:
            print(f'train_config.json not found in {local_dir}')

        dataset_repo = ''
        if train_config:
            dataset_repo = train_config.get(
                'dataset', {}
            ).get('repo_id', '')

        # Prepare tags
        tags = ['robotis', 'robotics']

        # Create ModelCardData with conditional datasets
        card_data_kwargs = {
            'license': 'apache-2.0',
            'tags': tags,
            'pipeline_tag': 'robotics',
        }
        if dataset_repo:
            card_data_kwargs['datasets'] = [dataset_repo]

        card_data = ModelCardData(**card_data_kwargs)

        # Get template path
        template_dir = Path(__file__).parent
        template_path = str(template_dir / 'model_card_template.md')

        # Create card from template
        card = ModelCard.from_template(
            card_data,
            template_path=template_path,
        )
        card.save(str(readme_path))
        print('Model README.md created using HuggingFace Hub')

    @staticmethod
    def _create_readme_if_not_exists(local_dir, repo_type):
        """
        Create README.md file if it doesn't exist in the folder.

        Uses HuggingFace Hub's DatasetCard or ModelCard.

        """
        readme_path = Path(local_dir) / 'README.md'

        if readme_path.exists():
            print(f'README.md already exists in {local_dir}')
            return

        print(f'Creating README.md in {local_dir}')

        try:
            if repo_type == 'dataset':
                DataManager._create_dataset_card(local_dir, readme_path)
        except Exception as e:
            print(f'Warning: Failed to create README.md: {e}')
            import traceback
            print(f'Traceback: {traceback.format_exc()}')

    @staticmethod
    def upload_huggingface_repo(
        repo_id,
        repo_type,
        local_dir,
        endpoint=None,
        token=None,
    ):
        try:
            api = HfApi(endpoint=endpoint, token=token)

            # Verify authentication first
            try:
                user_info = api.whoami()
                print(
                    f'Authenticated as: {user_info["name"]} '
                    f'({endpoint or "<default endpoint>"})'
                )
            except Exception as auth_e:
                print(f'Authentication failed: {auth_e}')
                print(
                    'Please make sure a valid token is registered for this '
                    f'endpoint: {endpoint or "<default>"}'
                )
                return False

            # Create repository
            print(f'Creating HuggingFace repository: {repo_id}')
            url = api.create_repo(
                repo_id,
                repo_type=repo_type,
                private=False,
                exist_ok=True,
            )
            print(f'Repository created/verified: {url}')

            # Delete .cache folder before upload
            DataManager._delete_dot_cache_folder_before_upload(local_dir)

            # Create README.md if it doesn't exist
            DataManager._create_readme_if_not_exists(
                local_dir, repo_type
            )

            print(f'Uploading folder {local_dir} to repository {repo_id}')

            # Capture stdout for logging
            from contextlib import redirect_stdout
            from .progress_tracker import HuggingFaceLogCapture

            # Use log capture with progress queue
            log_capture = HuggingFaceLogCapture(progress_queue=DataManager._progress_queue)

            with redirect_stdout(log_capture):
                # Upload folder contents via the HfApi instance so it picks up
                # the per-call endpoint+token without touching env vars.
                api.upload_large_folder(
                    repo_id=repo_id,
                    folder_path=local_dir,
                    repo_type=repo_type,
                    print_report=True,
                    print_report_every=1,
                )

            # Create tag
            if repo_type == 'dataset':
                try:
                    print(f'Creating tag for {repo_id} ({repo_type})')
                    api.create_tag(repo_id=repo_id, tag='v2.1', repo_type=repo_type)
                    print(f'Tag "v2.1" created successfully for {repo_id}')
                except Exception as e:
                    print(f'Warning: Failed to create tag for {repo_id} ({repo_type}): {e}')
                    # Don't fail the entire upload just because tag creation failed

            return True
        except Exception as e:
            print(f'Error Uploading HuggingFace repo: {e}')
            # Print more detailed error information
            import traceback
            print(f'Detailed error traceback:\n{traceback.format_exc()}')
            return False

    @staticmethod
    def _delete_dot_cache_folder_before_upload(local_dir):
        dot_cache_path = Path(local_dir) / '.cache'
        if dot_cache_path.exists():
            shutil.rmtree(dot_cache_path)
            print(f'Deleted {local_dir}/.cache folder before upload')

    @staticmethod
    def delete_huggingface_repo(
        repo_id,
        repo_type='dataset',
        endpoint=None,
        token=None,
    ):
        try:
            api = HfApi(endpoint=endpoint, token=token)
            return api.delete_repo(repo_id, repo_type=repo_type)
        except Exception as e:
            print(f'Error deleting HuggingFace repo: {e}')
            return False

    @staticmethod
    def get_huggingface_repo_list(
        author,
        data_type='dataset',
        endpoint=None,
        token=None,
    ):
        api = HfApi(endpoint=endpoint, token=token)
        repo_id_list = []
        if data_type == 'dataset':
            for dataset in api.list_datasets(author=author):
                repo_id_list.append(dataset.id)
        elif data_type == 'model':
            for model in api.list_models(author=author):
                repo_id_list.append(model.id)
        return repo_id_list[::-1]

    @staticmethod
    def get_collections_repo_list(
        collection_id,
        endpoint=None,
        token=None,
    ):
        api = HfApi(endpoint=endpoint, token=token)
        collection_list = api.get_collection(collection_id)
        return [item.item_id for item in collection_list.items]
