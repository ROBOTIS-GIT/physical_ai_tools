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

import gc
import json
import os
from pathlib import Path
import queue
import shutil
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from huggingface_hub import (
    DatasetCard,
    DatasetCardData,
    HfApi,
    ModelCard,
    ModelCardData,
    snapshot_download,
    upload_large_folder
)
from huggingface_hub.errors import LocalTokenNotFoundError
from physical_ai_interfaces.msg import TaskStatus
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

    def __init__(
            self,
            save_root_path,
            robot_type,
            task_info):
        self._robot_type = robot_type
        self._save_repo_name = f'{robot_type}_{task_info.task_name}'
        self._save_path = save_root_path / self._save_repo_name
        self._save_rosbag_path = '/workspace/rosbag2/' + self._save_repo_name
        self._single_task = len(task_info.task_instruction) == 1
        self._task_info = task_info

        # Find next available episode number from existing folders
        self._record_episode_count = self._find_next_episode_number()
        self._start_time_s = 0
        self._proceed_time = 0
        self._status = 'idle'  # Start in idle state (simplified mode)
        self._cpu_checker = CPUChecker()
        self.data_converter = DataConverter()
        self.current_instruction = ''
        self._current_task = 0
        self._init_task_limits()
        self._current_scenario_number = 0

    def _find_next_episode_number(self) -> int:
        """
        Find the next available episode number by scanning existing directories.

        Checks the rosbag save path for existing episode folders (0, 1, 2, ...)
        and returns the next available number.

        Returns:
            Next available episode number (0 if no existing episodes).
        """
        rosbag_dir = self._save_rosbag_path

        if not os.path.exists(rosbag_dir):
            print(f'[DataManager] No existing folder at {rosbag_dir}, starting from episode 0')
            return 0

        # Find all numeric folder names
        existing_episodes = []
        try:
            for item in os.listdir(rosbag_dir):
                item_path = os.path.join(rosbag_dir, item)
                if os.path.isdir(item_path) and item.isdigit():
                    existing_episodes.append(int(item))
        except OSError as e:
            print(f'[DataManager] Error scanning directory: {e}, starting from episode 0')
            return 0

        if not existing_episodes:
            print(f'[DataManager] No existing episodes in {rosbag_dir}, starting from episode 0')
            return 0

        next_episode = max(existing_episodes) + 1
        print(f'[DataManager] Found existing episodes {sorted(existing_episodes)}, '
              f'starting from episode {next_episode}')
        return next_episode

    def get_status(self):
        return self._status

    # ========== Simplified Recording Methods (rosbag2-only mode) ==========

    def start_recording(self):
        """
        Start recording (simplified mode).

        Changes status to 'recording' for rosbag to begin writing.
        """
        self._status = 'recording'
        self._start_time_s = time.perf_counter()
        self.current_instruction = self._task_info.task_instruction[0] \
            if self._task_info.task_instruction else ''
        print(f'[DataManager] Recording started - Episode {self._record_episode_count}')

    def stop_recording(self):
        """
        Stop recording and save (simplified mode).

        Changes status to 'idle' and increments episode count.
        """
        self._status = 'idle'
        self._record_episode_count += 1
        self._start_time_s = 0
        print(f'[DataManager] Recording stopped - Episode saved. '
              f'Total episodes: {self._record_episode_count}')

    def cancel_recording(self):
        """
        Cancel recording without saving (simplified mode).

        Changes status to 'idle' without incrementing episode count.
        """
        self._status = 'idle'
        self._start_time_s = 0
        print('[DataManager] Recording cancelled - Episode discarded')

    def is_recording(self):
        """Check if currently recording."""
        return self._status == 'recording'

    # ========== End Simplified Recording Methods ==========

    def get_save_rosbag_path(self):
        """Get rosbag save path for current episode."""
        # For simplified mode, return path when recording
        if self._status == 'idle':
            return None  # Not recording
        if self._status == 'warmup':
            return None  # Legacy: Not ready yet
        return self._save_rosbag_path + f'/{self._record_episode_count}'

    def save_robotis_metadata(self, urdf_path: str = None, needs_review: bool = False):
        """
        Save URDF and metadata for ROBOTIS format.

        Called after each episode rosbag is saved.
        Copies URDF file and all referenced mesh files.

        Args:
            urdf_path: Path to URDF file to copy.
            needs_review: If True, marks episode as needing review (e.g., cancelled recording).
        """
        rosbag_path = self.get_save_rosbag_path()
        if rosbag_path is None:
            return

        # Create rosbag directory if not exists
        os.makedirs(rosbag_path, exist_ok=True)

        # Copy URDF file and mesh files
        if urdf_path and os.path.exists(urdf_path):
            urdf_dest = os.path.join(rosbag_path, 'robot.urdf')
            try:
                # Copy URDF and mesh files with path conversion
                self._copy_urdf_with_meshes(urdf_path, urdf_dest, rosbag_path)
                print(f'[ROBOTIS] URDF and meshes copied to: {rosbag_path}')
            except Exception as e:
                print(f'[ROBOTIS] Failed to copy URDF/meshes: {e}')
                # Fallback: copy URDF only
                try:
                    shutil.copy2(urdf_path, urdf_dest)
                    print(f'[ROBOTIS] URDF copied (without meshes): {urdf_dest}')
                except Exception as e2:
                    print(f'[ROBOTIS] Failed to copy URDF: {e2}')

        # Save metadata JSON
        meta_data = {
            'task_instruction': self.current_instruction,
            'robot_type': self._robot_type,
            'episode_index': self._record_episode_count,
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'fps': self._task_info.fps if hasattr(self._task_info, 'fps') else 15,
            'format_version': 'robotis_v1',
            'device_serial': socket.gethostname(),
            'needs_review': needs_review,
        }

        meta_data_path = os.path.join(rosbag_path, 'episode_info.json')
        try:
            with open(meta_data_path, 'w') as f:
                json.dump(meta_data, f, indent=2)
            print(f'[ROBOTIS] Metadata saved to: {meta_data_path}')
        except Exception as e:
            print(f'[ROBOTIS] Failed to save metadata: {e}')

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
        current_status.task_info = self._task_info

        # Simplified mode statuses
        if self._status == 'idle':
            current_status.phase = TaskStatus.READY
            current_status.total_time = int(0)
        elif self._status == 'recording':
            current_status.phase = TaskStatus.RECORDING
            # Calculate elapsed time
            if self._start_time_s > 0:
                elapsed = time.perf_counter() - self._start_time_s
                self._proceed_time = int(elapsed)
            current_status.total_time = int(0)  # No time limit in simplified mode
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

    def _episode_reset(self):
        """Reset episode state for next recording."""
        self._start_time_s = 0
        gc.collect()

    def convert_action_to_joint_trajectory_msg(self, action):
        joint_trajectory_msgs = self.data_converter.tensor_array2joint_trajectory(
            action,
            self.total_joint_order)
        return joint_trajectory_msgs

    def get_task_info(self):
        return self._task_info

    def _init_task_limits(self):
        if not self._single_task:
            self._task_info.num_episodes = 1_000_000
            self._task_info.episode_time_s = 1_000_000

    @staticmethod
    def get_robot_type_from_info_json(info_json_path):
        with open(info_json_path, 'r', encoding='utf-8') as f:
            info = json.load(f)
        return info.get('robot_type', '')

    @staticmethod
    def get_huggingface_user_id(hf_endpoint='http://192.168.60.152:1000'):
        def api_call():
            # Set custom HF endpoint
            os.environ['HF_ENDPOINT'] = hf_endpoint
            print(f'[get_huggingface_user_id] Using HF endpoint: {hf_endpoint}')

            api = HfApi()
            try:
                user_info = api.whoami()
                user_ids = [user_info['name']]
                for org_info in user_info['orgs']:
                    user_ids.append(org_info['name'])
                return user_ids
            except LocalTokenNotFoundError as e:
                print(f'No registered HuggingFace token found: {e}')
                raise Exception('No registered HuggingFace token found')
            except Exception as e:
                print(f'Token validation failed: {e}')
                raise

        # Use queue to get result from thread
        result_queue = queue.Queue()

        def worker():
            try:
                result = api_call()
                result_queue.put(('success', result))
            except Exception as e:
                result_queue.put(('error', e))

        # Start thread and wait with timeout
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

        try:
            # Wait for result with 1.5 second timeout
            status, data = result_queue.get(timeout=1.5)
            if status == 'success':
                if data:
                    print(data)
                return data
            else:
                raise data
        except queue.Empty:
            print('Token validation timed out after 1.5 seconds')
            return None

    @staticmethod
    def register_huggingface_token(hf_token, hf_endpoint='http://192.168.60.152:1000'):
        def validate_and_save_token():
            # Set custom HF endpoint BEFORE any HF API calls
            os.environ['HF_ENDPOINT'] = hf_endpoint
            print(f'[register_huggingface_token] Using HF endpoint: {hf_endpoint}')

            try:
                # Verify the token works with custom endpoint
                api = HfApi(token=hf_token)
                user_info = api.whoami()
                user_name = user_info['name']
                print(f'Successfully validated HuggingFace token for user: {user_name}')

                # Save token using login() function
                from huggingface_hub import login
                login(token=hf_token, add_to_git_credential=False)

                # Verify token was saved
                from pathlib import Path
                token_path = Path.home() / ".cache" / "huggingface" / "token"
                print(f'Token saved successfully to {token_path}')

                return True
            except Exception as e:
                print(f'Token is invalid or save failed: {e}')
                import traceback
                traceback.print_exc()
                return False

        # Use queue to get result from thread
        result_queue = queue.Queue()

        def worker():
            result = validate_and_save_token()
            result_queue.put(result)

        # Start thread and wait with timeout
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

        try:
            # Wait for result with 1.5 second timeout
            is_valid = result_queue.get(timeout=1.5)
            if not is_valid:
                return False

            # Token validation and save successful
            print('Token validation and save completed successfully')
            return True

        except queue.Empty:
            print('Token validation timed out after 1.5 seconds')
            return False

    @staticmethod
    def download_huggingface_repo(
        repo_id,
        repo_type='dataset'
    ):
        download_path = {
            'dataset': Path.home() / '.cache/huggingface/lerobot',
            'model': Path.home() / 'ros2_ws/src/physical_ai_tools/lerobot/outputs/train/'
        }

        save_path = download_path.get(repo_type)

        if save_path is None:
            raise ValueError(f'Invalid repo type: {repo_type}')

        save_dir = save_path / repo_id

        try:
            print(f'Starting download of {repo_id} ({repo_type})...')

            # Create a wrapper class that includes the progress_queue
            class ProgressTqdmWrapper(HuggingFaceProgressTqdm):

                def __init__(self, *args, **kwargs):
                    kwargs['progress_queue'] = DataManager._progress_queue
                    super().__init__(*args, **kwargs)

            result = snapshot_download(
                repo_id=repo_id,
                repo_type=repo_type,
                local_dir=save_dir,
                tqdm_class=ProgressTqdmWrapper
            )

            print(f'Download completed: {repo_id}')
            return result
        except Exception as e:
            print(f'Error downloading HuggingFace repo: {e}')
            # Print more detailed error information
            import traceback
            print(f'Detailed error traceback:\n{traceback.format_exc()}')
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
    ):
        try:
            api = HfApi()

            # Verify authentication first
            try:
                user_info = api.whoami()
                print(f'Authenticated as: {user_info["name"]}')
            except Exception as auth_e:
                print(f'Authentication failed: {auth_e}')
                print('Please make sure you are authenticated with HuggingFace')
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
                # Upload folder contents
                upload_large_folder(
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
    ):
        try:
            result = HfApi().delete_repo(repo_id, repo_type=repo_type)
            return result
        except Exception as e:
            print(f'Error deleting HuggingFace repo: {e}')
            return False

    @staticmethod
    def get_huggingface_repo_list(
        author,
        data_type='dataset'
    ):
        repo_id_list = []
        if data_type == 'dataset':
            dataset_list = HfApi().list_datasets(author=author)
            for dataset in dataset_list:
                repo_id_list.append(dataset.id)

        elif data_type == 'model':
            model_list = HfApi().list_models(author=author)
            for model in model_list:
                repo_id_list.append(model.id)
        reverse = repo_id_list[::-1]
        return reverse

    @staticmethod
    def get_collections_repo_list(
        collection_id
    ):
        collection_list = HfApi().get_collection(collection_id)
        repo_list_in_collection = []
        for item in collection_list.items:
            repo_list_in_collection.append(item.item_id)
        return repo_list_in_collection

    @staticmethod
    def upload_rosbag_folders(
        folder_paths,
        hf_token,
        hf_endpoint='http://192.168.60.152:1000',
        progress_callback=None
    ):
        """
        Upload multiple rosbag folders to custom HuggingFace Hub.

        Args:
            folder_paths: List of rosbag folder paths to upload
            hf_token: HuggingFace authentication token
            hf_endpoint: Custom HF Hub endpoint (default: internal server)
            progress_callback: Optional callback function for progress updates
                             Signature: callback(folder_name, percentage, status, error=None)

        Returns:
            Dict with upload results:
            {
                'success': bool,
                'results': [
                    {
                        'folder_name': str,
                        'repo_id': str,
                        'success': bool,
                        'error': str (optional)
                    },
                    ...
                ]
            }
        """
        import os

        try:
            # Set custom HF endpoint
            os.environ['HF_ENDPOINT'] = hf_endpoint
            print(f'[upload_rosbag_folders] Using HF endpoint: {hf_endpoint}')

            # Initialize HuggingFace API with token
            # If hf_token is None, HfApi will use the saved token from ~/.cache/huggingface/token
            if hf_token is None:
                print('[upload_rosbag_folders] Using saved token from server')
                api = HfApi()  # Use saved token
            else:
                api = HfApi(token=hf_token)  # Use provided token

            # Verify authentication and get username
            try:
                user_info = api.whoami()
                username = user_info['name']
                print(f'[upload_rosbag_folders] Authenticated as: {username}')
            except Exception as auth_e:
                print(f'[upload_rosbag_folders] Authentication failed: {auth_e}')
                return {
                    'success': False,
                    'results': [],
                    'error': f'Authentication failed: {auth_e}'
                }

            results = []

            # Upload each folder
            for folder_path in folder_paths:
                folder_path_obj = Path(folder_path)
                folder_name = folder_path_obj.name
                repo_id = f"{username}/{folder_name}"

                print(f'[upload_rosbag_folders] Processing folder: {folder_name} -> {repo_id}')

                try:
                    # Notify upload start
                    if progress_callback:
                        progress_callback(folder_name, 0, 'starting')

                    # Create repository
                    print(f'[upload_rosbag_folders] Creating repository: {repo_id}')
                    api.create_repo(
                        repo_id=repo_id,
                        repo_type='dataset',
                        private=False,
                        exist_ok=True
                    )

                    # Notify upload in progress
                    if progress_callback:
                        progress_callback(folder_name, 10, 'uploading')

                    # Upload folder contents
                    print(f'[upload_rosbag_folders] Uploading folder: {folder_path}')
                    api.upload_folder(
                        folder_path=folder_path,
                        repo_id=repo_id,
                        repo_type='dataset',
                    )

                    # Notify completion
                    if progress_callback:
                        progress_callback(folder_name, 100, 'completed')

                    results.append({
                        'folder_name': folder_name,
                        'repo_id': repo_id,
                        'success': True,
                    })

                    print(f'[upload_rosbag_folders] Successfully uploaded {folder_name} to {repo_id}')

                except Exception as e:
                    error_msg = str(e)
                    print(f'[upload_rosbag_folders] Failed to upload {folder_name}: {error_msg}')

                    # Notify failure
                    if progress_callback:
                        progress_callback(folder_name, 0, 'failed', error=error_msg)

                    results.append({
                        'folder_name': folder_name,
                        'repo_id': repo_id,
                        'success': False,
                        'error': error_msg,
                    })

            # Return overall result
            all_success = all(r['success'] for r in results)
            return {
                'success': all_success,
                'results': results,
            }

        except Exception as e:
            print(f'[upload_rosbag_folders] Unexpected error: {e}')
            import traceback
            print(f'Traceback:\n{traceback.format_exc()}')
            return {
                'success': False,
                'results': [],
                'error': str(e)
            }
