// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

// TaskPhase enum-like object for task status phases
// Use this for better code readability and maintainability

const TaskCommand = {
  NONE: 0,
  START_RECORD: 1,
  START_INFERENCE: 2,
  STOP: 3,
  NEXT: 4,
  RERECORD: 5,
  FINISH: 6,
  SKIP_TASK: 7,
};

export default TaskCommand;
