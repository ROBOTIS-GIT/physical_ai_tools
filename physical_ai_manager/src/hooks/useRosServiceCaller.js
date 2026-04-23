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

import { useCallback } from 'react';
import { useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import PageType from '../constants/pageType';
import TaskCommand from '../constants/taskCommand';
import TrainingCommand from '../constants/trainingCommand';
import EditDatasetCommand from '../constants/commands';
import rosConnectionManager from '../utils/rosConnectionManager';
import { DEFAULT_PATHS } from '../constants/paths';

export function useRosServiceCaller() {
  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const pendingPrimitive = useSelector((state) => state.tasks.pendingPrimitive);
  const trainingInfo = useSelector((state) => state.training.trainingInfo);
  const trainingResumePolicyPath = useSelector((state) => state.training.resumePolicyPath);
  const editDatasetInfo = useSelector((state) => state.editDataset);
  const page = useSelector((state) => state.ui.currentPage);
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);

  const callService = useCallback(
    async (serviceName, serviceType, request, timeoutMs = 10000) => {
      try {
        const ros = await rosConnectionManager.getConnection(rosbridgeUrl);

        // Additional check for connection health
        if (!ros || !ros.isConnected) {
          throw new Error('ROS connection is not available or not connected');
        }

        return new Promise((resolve, reject) => {
          const service = new ROSLIB.Service({
            ros,
            name: serviceName,
            serviceType: serviceType,
          });
          const req = new ROSLIB.ServiceRequest(request);

          // Set a timeout for the service call
          const serviceTimeout = setTimeout(() => {
            reject(new Error(`Service call timeout for ${serviceName}`));
          }, timeoutMs);

          service.callService(
            req,
            (result) => {
              clearTimeout(serviceTimeout);
              resolve(result);
            },
            (error) => {
              clearTimeout(serviceTimeout);
              console.error('Service call failed:', error);
              reject(
                new Error(`Service call failed for ${serviceName}: ${error.message || error}`)
              );
            }
          );
        });
      } catch (error) {
        console.error('Failed to establish ROS connection for service call:', error);
        throw new Error(
          `ROS connection failed for service ${serviceName}: ${error.message || error}`
        );
      }
    },
    [rosbridgeUrl]
  );

  const sendRecordCommand = useCallback(
    async (command, options = {}) => {
      try {
        const commandEnumMap = {
          none: TaskCommand.NONE,
          start_record: TaskCommand.START_RECORD,
          start_inference: TaskCommand.START_INFERENCE,
          stop: TaskCommand.STOP,
          next: TaskCommand.NEXT,
          skip_task: TaskCommand.SKIP_TASK,
          rerecord: TaskCommand.RERECORD,
          finish: TaskCommand.FINISH,
          cancel: TaskCommand.CANCEL,
          convert_mp4: TaskCommand.CONVERT_MP4,
          stop_inference: TaskCommand.STOP_INFERENCE,
          resume_inference: TaskCommand.RESUME_INFERENCE,
          start_inference_record: TaskCommand.START_INFERENCE_RECORD,
          stop_inference_record: TaskCommand.STOP_INFERENCE_RECORD,
          cancel_inference_record: TaskCommand.CANCEL_INFERENCE_RECORD,
          start_segment: TaskCommand.START_SEGMENT,
          stop_segment: TaskCommand.STOP_SEGMENT,
          discard_segment: TaskCommand.DISCARD_SEGMENT,
          finish_episode: TaskCommand.FINISH_EPISODE,
          discard_episode: TaskCommand.DISCARD_EPISODE,
          set_task_info: TaskCommand.SET_TASK_INFO,
          cancel_segment: TaskCommand.CANCEL_SEGMENT,
          refresh_topics: TaskCommand.REFRESH_TOPICS,
        };
        const command_enum = commandEnumMap[command];
        if (command_enum === undefined) {
          throw new Error(`Unknown command: ${command}`);
        }

        let taskType = '';

        if (page === PageType.RECORD) {
          taskType = 'record';
        } else if (page === PageType.INFERENCE) {
          taskType = 'inference';
        }

        // Auto-fill taskName and taskInstruction if empty
        let taskName = taskInfo.taskName || '';
        let taskInstruction = (taskInfo.taskInstruction || []).filter(
          (instruction) => instruction.trim() !== ''
        );

        if (!taskName.trim()) {
          const now = new Date();
          const pad = (n) => String(n).padStart(2, '0');
          const yy = String(now.getFullYear()).slice(2);
          taskName = `task_${yy}${pad(now.getMonth() + 1)}${pad(now.getDate())}${pad(now.getHours())}${pad(now.getMinutes())}`;
        }
        if (taskInstruction.length === 0) {
          taskInstruction = [taskName];
        }

        const primitiveDescription = String(
          options.primitiveDescription !== undefined
            ? options.primitiveDescription
            : pendingPrimitive || ''
        );

        const request = {
          task_info: {
            task_num: String(taskInfo.taskNum || ''),
            task_name: String(taskName),
            task_type: String(taskType),
            task_instruction: taskInstruction,
            primitive_description: primitiveDescription,
            policy_path: String(taskInfo.policyPath || ''),
            record_inference_mode: Boolean(taskInfo.recordInferenceMode),
            tags: [],
            control_hz: Number(taskInfo.controlHz || 100),
            inference_hz: Number(taskInfo.inferenceHz || 15),
            chunk_align_window_s: Number(
              taskInfo.chunkAlignWindowS !== '' && taskInfo.chunkAlignWindowS != null
                ? taskInfo.chunkAlignWindowS
                : 0.3
            ),
          },
          command: Number(command_enum),
          segment_index: Number(options.segmentIndex || 0),
          episode_index: Number(options.episodeIndex || 0),
        };


        const result = await callService(
          '/task/command',
          'physical_ai_interfaces/srv/SendCommand',
          request
        );

        return result;
      } catch (error) {
        console.error(`Error in sendRecordCommand for '${command}':`, error);
        // Re-throw with more context
        throw new Error(`${error.message || error}`);
      }
    },
    [callService, taskInfo, pendingPrimitive, page]
  );

  const getImageTopicList = useCallback(async () => {
    try {
      const result = await callService(
        '/image/get_available_list',
        'physical_ai_interfaces/srv/GetImageTopicList',
        {}
      );
      return result;
    } catch (error) {
      console.error('Failed to get image topic list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const getRobotTypeList = useCallback(async () => {
    try {
      const result = await callService(
        '/get_robot_types',
        'physical_ai_interfaces/srv/GetRobotTypeList',
        {}
      );
      return result;
    } catch (error) {
      console.error('Failed to get robot type list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const setRobotType = useCallback(
    async (robot_type) => {
      try {

        const result = await callService(
          '/set_robot_type',
          'physical_ai_interfaces/srv/SetRobotType',
          { robot_type: robot_type }
        );

        return result;
      } catch (error) {
        console.error('Failed to set robot type:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  // Register a token for a specific HuggingFace endpoint. The server validates
  // the (endpoint, token) pair via whoami() before persisting it, so a bad
  // token never reaches the on-disk store.
  const registerHFUser = useCallback(
    async ({ endpoint, label = '', token }) => {
      try {
        console.log('Calling service /register_hf_user with request:', {
          endpoint,
          label,
          token: '<redacted>',
        });

        const result = await callService(
          '/register_hf_user',
          'physical_ai_interfaces/srv/SetHFUser',
          { endpoint: endpoint || '', label: label || '', token: token || '' }
        );

        return result;
      } catch (error) {
        console.error('Failed to register HF user:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  // Fetch the user list for a given endpoint (empty endpoint = currently
  // active endpoint on the server side).
  const getRegisteredHFUser = useCallback(
    async (endpoint = '') => {
      try {
        console.log('Calling service /get_registered_hf_user with request:', {
          endpoint,
        });

        const result = await callService(
          '/get_registered_hf_user',
          'physical_ai_interfaces/srv/GetHFUser',
          { endpoint },
          3000
        );

        return result;
      } catch (error) {
        console.error('Failed to get registered HF user:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  // Return the full list of registered HF endpoints + the active one.
  const listHFEndpoints = useCallback(async () => {
    try {
      const result = await callService(
        '/huggingface/list_endpoints',
        'physical_ai_interfaces/srv/HFEndpointList',
        {},
        3000
      );
      return result;
    } catch (error) {
      console.error('Failed to list HF endpoints:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  // Set the server-side active endpoint. Empty string clears the selection.
  const selectHFEndpoint = useCallback(
    async (endpoint) => {
      try {
        const result = await callService(
          '/huggingface/select_endpoint',
          'physical_ai_interfaces/srv/SelectHFEndpoint',
          { endpoint: endpoint || '' }
        );
        return result;
      } catch (error) {
        console.error('Failed to select HF endpoint:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const getUserList = useCallback(async () => {
    try {

      const result = await callService(
        '/training/get_user_list',
        'physical_ai_interfaces/srv/GetUserList',
        {}
      );

      return result;
    } catch (error) {
      console.error('Failed to get user list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const getDatasetList = useCallback(
    async (user_id) => {
      try {
        console.log('Calling service /training/get_dataset_list with request:', {
          user_id: user_id,
        });

        const result = await callService(
          '/training/get_dataset_list',
          'physical_ai_interfaces/srv/GetDatasetList',
          { user_id: user_id }
        );

        return result;
      } catch (error) {
        console.error('Failed to get dataset list:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const getPolicyList = useCallback(async () => {
    try {

      const result = await callService(
        '/training/get_available_policy',
        'physical_ai_interfaces/srv/GetPolicyList',
        {}
      );

      return result;
    } catch (error) {
      console.error('Failed to get policy list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const getModelWeightList = useCallback(async () => {
    try {

      const result = await callService(
        '/training/get_model_weight_list',
        'physical_ai_interfaces/srv/GetModelWeightList',
        {}
      );

      return result;
    } catch (error) {
      console.error('Failed to get model weight list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const sendTrainingCommand = useCallback(
    async (command) => {
      try {
        let command_enum;
        switch (command) {
          case 'start':
            command_enum = TrainingCommand.START;
            break;
          case 'resume':
            command_enum = TrainingCommand.START;
            break;
          case 'finish':
            command_enum = TrainingCommand.FINISH;
            break;
          default:
            throw new Error(`Unknown command: ${command}`);
        }

        // Get relative path after base path
        const getRelativePath = (fullPath) => {
          const REQUIRED_BASE_PATH = DEFAULT_PATHS.POLICY_MODEL_PATH;

          if (!fullPath) return '';
          if (fullPath.startsWith(REQUIRED_BASE_PATH)) {
            return fullPath.substring(REQUIRED_BASE_PATH.length);
          }
          return fullPath;
        };

        const request = {
          command: command_enum,
          training_info: {
            dataset: trainingInfo.datasetRepoId,
            policy_type: trainingInfo.policyType,
            policy_device: trainingInfo.policyDevice,
            output_folder_name: trainingInfo.outputFolderName,
            seed: trainingInfo.seed,
            num_workers: trainingInfo.numWorkers,
            batch_size: trainingInfo.batchSize,
            steps: trainingInfo.steps,
            eval_freq: trainingInfo.evalFreq,
            log_freq: trainingInfo.logFreq,
            save_freq: trainingInfo.saveFreq,
          },
          resume: command === 'resume',
          resume_model_path: command === 'resume' ? getRelativePath(trainingResumePolicyPath) : '',
        };


        const result = await callService(
          '/training/command',
          'physical_ai_interfaces/srv/SendTrainingCommand',
          request
        );

        return result;
      } catch (error) {
        console.error('Failed to send training command:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService, trainingInfo, trainingResumePolicyPath]
  );

  const browseFile = useCallback(
    async (action, currentPath = '', targetName = '', targetFiles = null, targetFolders = null) => {
      try {
        // Ensure target_files is always an array
        let filesArray = [];
        if (targetFiles) {
          filesArray = Array.isArray(targetFiles) ? targetFiles : [targetFiles];
        }

        // Ensure target_folders is always an array
        let foldersArray = [];
        if (targetFolders) {
          foldersArray = Array.isArray(targetFolders) ? targetFolders : [targetFolders];
        }

        const requestData = {
          action: action,
          current_path: currentPath,
          target_name: targetName,
          target_files: filesArray,
          target_folders: foldersArray,
        };

        const result = await callService(
          '/browse_file',
          'physical_ai_interfaces/srv/BrowseFile',
          requestData
        );

        return result;
      } catch (error) {
        console.error('Failed to browse file:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const sendEditDatasetCommand = useCallback(
    async (command) => {
      try {
        console.log('Calling service /dataset/edit with request:', {
          command: command,
          edit_dataset_info: editDatasetInfo,
        });

        let command_enum;
        switch (command) {
          case 'merge':
            command_enum = EditDatasetCommand.MERGE;
            break;
          case 'delete':
            command_enum = EditDatasetCommand.DELETE;
            break;
          default:
            throw new Error(`Unknown command: ${command}`);
        }


        // Build merge_output_task_dir from mergeOutputPath + mergeOutputFolderName.
        let mergeOutputPath = editDatasetInfo.mergeOutputPath || '';
        if (mergeOutputPath.endsWith('/')) {
          mergeOutputPath = mergeOutputPath.slice(0, -1);
        }
        const merge_output_task_dir =
          mergeOutputPath && editDatasetInfo.mergeOutputFolderName
            ? `${mergeOutputPath}/${editDatasetInfo.mergeOutputFolderName}`
            : '';

        const result = await callService(
          '/dataset/edit',
          'physical_ai_interfaces/srv/EditDataset',
          {
            mode: command_enum,
            merge_source_task_dirs: editDatasetInfo.mergeSourceTaskDirs || [],
            merge_output_task_dir,
            merge_move_sources: Boolean(editDatasetInfo.mergeMoveSources),
            delete_task_dir: editDatasetInfo.deleteTaskDir || '',
            delete_episode_num: editDatasetInfo.deleteEpisodeNums || [],
            delete_compact:
              editDatasetInfo.deleteCompact === undefined
                ? true
                : Boolean(editDatasetInfo.deleteCompact),
          }
        );

        return result;
      } catch (error) {
        console.error('Failed to send edit dataset command:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService, editDatasetInfo]
  );

  const getDatasetInfo = useCallback(
    async (datasetPath) => {
      try {
        const result = await callService(
          '/dataset/get_info',
          'physical_ai_interfaces/srv/GetDatasetInfo',
          { dataset_path: datasetPath }
        );
        return result;
      } catch (error) {
        console.error('Failed to get dataset info:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const controlHfServer = useCallback(
    async (mode, repoId = '', repoType = '', localDir = '', endpoint = '') => {
      try {
        console.log('Calling service /huggingface/control with request:', {
          mode: mode,
          repo_id: repoId,
          repo_type: repoType,
          local_dir: localDir,
          endpoint: endpoint,
        });

        const request = {
          mode: mode,
          repo_id: repoId,
          repo_type: repoType,
          endpoint: endpoint || '',
        };

        // Only add local_dir if it's provided and not empty
        if (localDir && localDir.trim() !== '') {
          request.local_dir = localDir;
        }

        const result = await callService(
          '/huggingface/control',
          'physical_ai_interfaces/srv/ControlHfServer',
          request
        );

        return result;
      } catch (error) {
        console.error('Failed to control HF server:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const getTrainingInfo = useCallback(
    async (trainConfigPath) => {
      try {
        console.log('Calling service /training/get_training_info with request:', {
          train_config_path: trainConfigPath,
        });

        const result = await callService(
          '/training/get_training_info',
          'physical_ai_interfaces/srv/GetTrainingInfo',
          { train_config_path: trainConfigPath }
        );
        return result;
      } catch (error) {
        console.error('Failed to get training info:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const getReplayData = useCallback(
    async (bagPath) => {
      try {
        const apiUrl = `/api/replay-data${bagPath}`;

        const response = await fetch(apiUrl);

        if (!response.ok) {
          throw new Error(`HTTP error: ${response.status} ${response.statusText}`);
        }

        const result = await response.json();

        // Transform to match the expected format from ROS service
        return {
          success: result.success,
          message: result.message,
          video_files: result.video_files || [],
          video_topics: result.video_topics || [],
          video_names: result.video_names || [],
          video_fps: result.video_fps || [],
          frame_indices: result.frame_indices || [],
          frame_timestamps: result.frame_timestamps || [],
          joint_timestamps: result.joint_timestamps || [],
          joint_names: result.joint_names || [],
          joint_positions: result.joint_positions || [],
          action_timestamps: result.action_timestamps || [],
          action_names: result.action_names || [],
          action_values: result.action_values || [],
          start_time: result.start_time || 0,
          end_time: result.end_time || 0,
          duration: result.duration || 0,
          bag_path: bagPath,
          // Extended metadata
          robot_type: result.robot_type || '',
          recording_date: result.recording_date || null,
          file_size_bytes: result.file_size_bytes || 0,
          task_markers: result.task_markers || [],
          trim_points: result.trim_points || null,
          exclude_regions: result.exclude_regions || [],
          frame_counts: result.frame_counts || {},
          // MCAP direct streaming
          has_raw_images: result.has_raw_images || false,
          raw_image_topics: result.raw_image_topics || [],
          mcap_file: result.mcap_file || '',
        };
      } catch (error) {
        console.error('Failed to get replay data:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    []
  );

  const getRosbagList = useCallback(
    async (folderPath) => {
      try {
        const apiUrl = `/api/rosbag-list${folderPath}`;

        const response = await fetch(apiUrl);

        if (!response.ok) {
          throw new Error(`HTTP error: ${response.status} ${response.statusText}`);
        }

        const result = await response.json();

        return result;
      } catch (error) {
        console.error('Failed to get rosbag list:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    []
  );

  return {
    callService,
    sendRecordCommand,
    getImageTopicList,
    getRobotTypeList,
    setRobotType,
    registerHFUser,
    getRegisteredHFUser,
    listHFEndpoints,
    selectHFEndpoint,
    getUserList,
    getDatasetList,
    getPolicyList,
    getModelWeightList,
    sendTrainingCommand,
    browseFile,
    sendEditDatasetCommand,
    getDatasetInfo,
    controlHfServer,
    getTrainingInfo,
    getReplayData,
    getRosbagList,
  };
}
