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

const TaskPhase = {
  READY: 0,
  WARMING_UP: 1,
  RESETTING: 2,
  RECORDING: 3,
  SAVING: 4,
  STOPPED: 5,
  // Add more phases as needed
};

export default TaskPhase;
