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
# Author: Kiwoong Park


"""File browser utility class for handling file system operations."""

from concurrent.futures import as_completed, ThreadPoolExecutor
import datetime
import os
from typing import Dict, List, Optional, Set


class FileBrowseUtils:
    """Utility class for file browsing operations."""

    def __init__(self, max_workers: int = 4, logger=None):
        """Initialize the FileBrowseUtils instance."""
        self.max_workers = max_workers
        self.logger = logger

    def handle_get_path_action(self, current_path):
        """Handle get_path action and return path information."""
        current_path = current_path or os.path.expanduser('~')
        current_path = os.path.abspath(current_path)

        return {
            'success': True,
            'message': 'Current path retrieved successfully',
            'current_path': current_path,
            'parent_path': os.path.dirname(current_path),
            'selected_path': '',
            'items': []
        }

    def handle_go_parent_action(self, current_path):
        """Handle go_parent action and return parent directory information."""
        current_path = current_path or os.path.expanduser('~')
        parent_path = os.path.dirname(os.path.abspath(current_path))

        if os.path.exists(parent_path) and os.path.isdir(parent_path):
            return {
                'success': True,
                'message': 'Navigated to parent directory',
                'current_path': parent_path,
                'parent_path': os.path.dirname(parent_path),
                'selected_path': '',
                'items': self._get_directory_items(parent_path)
            }
        else:
            return {
                'success': False,
                'message': 'Cannot navigate to parent directory',
                'current_path': current_path,
                'parent_path': '',
                'selected_path': '',
                'items': []
            }

    def handle_go_parent_with_target_check(self,
                                           current_path: str,
                                           target_files: Set[str] = None,
                                           target_folders: Set[str] = None) -> Dict:
        """Handle go_parent action with parallel checking for target files/folders."""
        if current_path is None or current_path == '':
            current_path = os.path.expanduser('~')

        parent_path = os.path.dirname(os.path.abspath(current_path))

        if os.path.exists(parent_path) and os.path.isdir(parent_path):
            try:
                items = self._get_directories_with_target_check(
                    parent_path, target_files, target_folders)

                return {
                    'success': True,
                    'message': 'Navigated to parent directory with target check',
                    'current_path': parent_path,
                    'parent_path': os.path.dirname(parent_path),
                    'selected_path': '',
                    'items': items
                }
            except Exception as e:
                error_msg = f'Error during parent navigation with target check: {str(e)}'
                return {
                    'success': False,
                    'message': error_msg,
                    'current_path': current_path,
                    'parent_path': '',
                    'selected_path': '',
                    'items': []
                }
        else:
            return {
                'success': False,
                'message': 'Cannot navigate to parent directory',
                'current_path': current_path,
                'parent_path': '',
                'selected_path': '',
                'items': []
            }

    def handle_browse_action(self, current_path, target_name=None):
        """Handle browse action for directory or file selection."""
        current_path = current_path or os.path.expanduser('~')
        current_path = os.path.abspath(current_path)

        if target_name:
            return self._handle_target_selection(current_path, target_name)
        else:
            return self._handle_directory_browse(current_path)

    def handle_browse_with_target_check(self,
                                        current_path: str,
                                        target_name: str,
                                        target_files: Set[str] = None,
                                        target_folders: Set[str] = None) -> Dict:
        """Handle browse action with parallel checking for target files/folders."""
        if current_path is None or current_path == '':
            current_path = os.path.expanduser('~')

        if target_name:
            # Handle target selection (navigate to specific item)
            # with parallel target checking
            target_path = os.path.join(current_path, target_name)

            if os.path.exists(target_path) and os.path.isdir(target_path):
                # Navigate into directory and check for target files/folders
                try:
                    items = self._get_directories_with_target_check(
                        target_path, target_files, target_folders)

                    return {
                        'success': True,
                        'message': f'Navigated to {target_name}',
                        'current_path': target_path,
                        'parent_path': current_path,
                        'selected_path': target_path,
                        'items': items
                    }
                except Exception as e:
                    error_msg = f'Error during navigation with target check: {str(e)}'
                    return {
                        'success': False,
                        'message': error_msg,
                        'current_path': current_path,
                        'parent_path': '',
                        'selected_path': '',
                        'items': []
                    }
            else:
                # File selection or non-existent path - use standard logic
                return self.handle_browse_action(current_path, target_name)
        else:
            # Handle directory browsing with parallel target checking
            try:
                items = self._get_directories_with_target_check(
                    current_path, target_files, target_folders)

                return {
                    'success': True,
                    'message': 'Directory browsed successfully with target check',
                    'current_path': current_path,
                    'parent_path': self._get_parent_path(current_path),
                    'selected_path': '',
                    'items': items
                }
            except Exception as e:
                return {
                    'success': False,
                    'message': f'Error during target check: {str(e)}',
                    'current_path': current_path,
                    'parent_path': '',
                    'selected_path': '',
                    'items': []
                }

    def _handle_target_selection(self, current_path, target_name):
        """Handle target file/directory selection."""
        target_path = os.path.join(current_path, target_name)

        if os.path.exists(target_path):
            if os.path.isdir(target_path):
                # Navigate into directory
                return {
                    'success': True,
                    'message': f'Navigated to {target_name}',
                    'current_path': target_path,
                    'parent_path': current_path,
                    'selected_path': target_path,
                    'items': self._get_directory_items(target_path)
                }
            else:
                # Select file
                return {
                    'success': True,
                    'message': f'Selected file {target_name}',
                    'current_path': current_path,
                    'parent_path': os.path.dirname(current_path),
                    'selected_path': target_path,
                    'items': self._get_directory_items(current_path)
                }
        else:
            return {
                'success': False,
                'message': f'Item {target_name} not found',
                'current_path': current_path,
                'parent_path': os.path.dirname(current_path),
                'selected_path': '',
                'items': self._get_directory_items(current_path)
            }

    def _handle_directory_browse(self, current_path):
        """Handle directory browsing."""
        if os.path.exists(current_path) and os.path.isdir(current_path):
            return {
                'success': True,
                'message': 'Directory browsed successfully',
                'current_path': current_path,
                'parent_path': os.path.dirname(current_path),
                'selected_path': '',
                'items': self._get_directory_items(current_path)
            }
        else:
            # Return error if directory doesn't exist
            return {
                'success': False,
                'message': f'Directory does not exist: {current_path}',
                'current_path': '',
                'parent_path': '',
                'selected_path': '',
                'items': []
            }

    def _check_directories_for_targets(
        self,
        directory_paths: List[str],
        target_files: Set[str] = None,
        target_folders: Set[str] = None
    ) -> Dict[str, bool]:
        """Check multiple directories in parallel for presence of ALL target files and folders."""
        def check_single_directory(dir_path: str) -> tuple:
            """Check if ALL target files and folders exist in a single directory."""
            try:
                found_files = set()
                found_folders = set()

                # Collect all files and folders in the directory
                with os.scandir(dir_path) as entries:
                    for entry in entries:
                        entry_name = entry.name

                        if entry.is_file(follow_symlinks=False):
                            found_files.add(entry_name)
                        elif entry.is_dir(follow_symlinks=False):
                            found_folders.add(entry_name)

                # Check if ALL required targets are found
                files_satisfied = not target_files or target_files.issubset(found_files)
                folders_satisfied = not target_folders or target_folders.issubset(found_folders)

                return (dir_path, files_satisfied and folders_satisfied)

            except (OSError, PermissionError):
                return (dir_path, False)

        results = {}

        # Use ThreadPoolExecutor for I/O bound operations
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            # Submit all directory checks
            future_to_path = {
                executor.submit(check_single_directory, path): path
                for path in directory_paths
            }

            # Collect results
            for future in as_completed(future_to_path):
                try:
                    dir_path, has_target = future.result()
                    results[dir_path] = has_target
                except Exception as e:
                    # If error occurs, assume no target file/folder
                    path = future_to_path[future]
                    if self.logger:
                        error_msg = f'Error during directory check: {path} {e}'
                        self.logger.error(error_msg)
                    results[path] = False

        return results

    def _get_directories_with_target_check(
        self,
        directory_path: str,
        target_files: Optional[Set[str]] = None,
        target_folders: Optional[Set[str]] = None
    ) -> List[Dict]:
        """Get directory items with parallel target file/folder existence checking."""
        items = self._get_directory_items(directory_path)

        if target_files is None and target_folders is None:
            return items

        # Separate directories from files
        directories = [item for item in items if item['is_directory']]
        files = [item for item in items if not item['is_directory']]

        if not directories:
            return items

        # Check directories in parallel for target files/folders
        dir_paths = [item['full_path'] for item in directories]
        check_results = self._check_directories_for_targets(
            dir_paths, target_files, target_folders)

        # Add has_target_file field to directory items
        for item in directories:
            item['has_target_file'] = check_results.get(item['full_path'], False)

        # Files don't have has_target_file field (or set to False)
        for item in files:
            item['has_target_file'] = False

        return directories + files

    def _get_directory_items(self, directory_path):
        """Get list of items in the directory as dictionaries."""
        items = []

        try:
            if (not os.path.exists(directory_path)
                    or not os.path.isdir(directory_path)):
                return items

            with os.scandir(directory_path) as it:
                for entry in it:
                    try:
                        name = entry.name
                        # Skip hidden files and directories except .cache
                        if name.startswith('.') and name != '.cache':
                            continue

                        is_directory = entry.is_dir(follow_symlinks=False)
                        if is_directory:
                            size = -1
                            # Use entry.stat for mtime without extra os.path.* calls.
                            mtime = entry.stat(follow_symlinks=False).st_mtime
                        else:
                            stat_result = entry.stat(follow_symlinks=False)
                            size = stat_result.st_size
                            mtime = stat_result.st_mtime

                        timestamp = datetime.datetime.fromtimestamp(mtime)
                        modified_time = timestamp.strftime('%Y-%m-%d %H:%M:%S')

                        item_dict = {
                            'name': name,
                            'full_path': os.path.join(directory_path, name),
                            'is_directory': is_directory,
                            'size': size,
                            'modified_time': modified_time
                        }
                        items.append(item_dict)
                    except (OSError, PermissionError):
                        # Skip items that cannot be accessed
                        continue
        except (OSError, PermissionError):
            # Return empty list if directory cannot be read
            pass

        # Sort items by name to keep previous behavior deterministic.
        try:
            items.sort(key=lambda x: x['name'])
        except Exception:
            # If anything unexpected happens during sort, return unsorted items.
            if self.logger:
                self.logger.error(f'Error during sort: {items}')
            pass

        return items

    def _get_parent_path(self, path: str) -> str:
        """Get parent directory path."""
        return os.path.dirname(os.path.abspath(path))
