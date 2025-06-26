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
# Author: Dongyun Kim


import json
import os


def read_json_file(file_path: str) -> dict:
    try:
        if os.path.exists(file_path) and not os.path.isfile(file_path):
            return None
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f'File not found: {file_path}')
        return None
    except json.JSONDecodeError:
        print(f'Error decoding JSON from file: {file_path}. The file may be empty or malformed.')
        return None
    except Exception as e:
        print(f'An error occurred while reading the file: {file_path}. Error: {e}')
        return None
