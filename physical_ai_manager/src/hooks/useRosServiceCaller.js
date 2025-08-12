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

export function useRosServiceCaller() {
  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const trainingInfo = useSelector((state) => state.training.trainingInfo);
  const editDatasetInfo = useSelector((state) => state.editDataset);
  const page = useSelector((state) => state.ui.currentPage);
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);

  const callService = useCallback(
    async (serviceName, serviceType, request) => {
      try {
        console.log(`Attempting to call service: ${serviceName}`);
        const ros = await rosConnectionManager.getConnection(rosbridgeUrl);

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
          }, 10000); // 10 second timeout

          service.callService(
            req,
            (result) => {
              clearTimeout(serviceTimeout);
              console.log('Service call successful:', result);
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
    async (command) => {
      try {
        let command_enum;
        switch (command) {
          case 'none':
            command_enum = TaskCommand.NONE;
            break;
          case 'start_record':
            command_enum = TaskCommand.START_RECORD;
            break;
          case 'start_inference':
            command_enum = TaskCommand.START_INFERENCE;
            break;
          case 'stop':
            command_enum = TaskCommand.STOP;
            break;
          case 'next':
            command_enum = TaskCommand.NEXT;
            break;
          case 'skip_task':
            command_enum = TaskCommand.SKIP_TASK;
            break;
          case 'rerecord':
            command_enum = TaskCommand.RERECORD;
            break;
          case 'finish':
            command_enum = TaskCommand.FINISH;
            break;
          default:
            throw new Error(`Unknown command: ${command}`);
        }

        let taskType = '';

        if (page === PageType.RECORD) {
          taskType = 'record';
        } else if (page === PageType.INFERENCE) {
          taskType = 'inference';
        }

        const task_instruction = taskInfo.taskInstruction.filter(
          (instruction) => instruction.trim() !== ''
        );

        const request = {
          task_info: {
            task_name: String(taskInfo.taskName || ''),
            task_type: String(taskType),
            user_id: String(taskInfo.userId || ''),
            task_instruction: task_instruction,
            policy_path: String(taskInfo.policyPath || ''),
            record_inference_mode: Boolean(taskInfo.recordInferenceMode),
            fps: Number(taskInfo.fps) || 0,
            tags: taskInfo.tags || [],
            warmup_time_s: Number(taskInfo.warmupTime) || 0,
            episode_time_s: Number(taskInfo.episodeTime) || 0,
            reset_time_s: Number(taskInfo.resetTime) || 0,
            num_episodes: Number(taskInfo.numEpisodes) || 0,
            push_to_hub: Boolean(taskInfo.pushToHub),
            private_mode: Boolean(taskInfo.privateMode),
            use_optimized_save_mode: Boolean(taskInfo.useOptimizedSave),
          },
          command: Number(command_enum),
        };

        console.log('request:', request);

        console.log(`Sending command '${command}' (${command_enum}) to service`);
        const result = await callService(
          '/task/command',
          'physical_ai_interfaces/srv/SendCommand',
          request
        );

        console.log(`Service response for command '${command}':`, result);
        return result;
      } catch (error) {
        console.error(`Error in sendRecordCommand for '${command}':`, error);
        // Re-throw with more context
        throw new Error(`${error.message || error}`);
      }
    },
    [callService, taskInfo, page]
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
        console.log('setRobotType called with:', robot_type);
        console.log('Calling service /set_robot_type with request:', { robot_type: robot_type });

        const result = await callService(
          '/set_robot_type',
          'physical_ai_interfaces/srv/SetRobotType',
          { robot_type: robot_type }
        );

        console.log('setRobotType service response:', result);
        return result;
      } catch (error) {
        console.error('Failed to set robot type:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const registerHFUser = useCallback(
    async (token) => {
      try {
        console.log('Calling service /register_hf_user with request:', { token: token });

        const result = await callService(
          '/register_hf_user',
          'physical_ai_interfaces/srv/SetHFUser',
          { token: token }
        );

        console.log('registerHFUser service response:', result);
        return result;
      } catch (error) {
        console.error('Failed to register HF user:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService]
  );

  const getRegisteredHFUser = useCallback(async () => {
    try {
      console.log('Calling service /get_registered_hf_user with request:', {});

      const result = await callService(
        '/get_registered_hf_user',
        'physical_ai_interfaces/srv/GetHFUser',
        {}
      );

      console.log('getRegisteredHFUser service response:', result);
      return result;
    } catch (error) {
      console.error('Failed to get registered HF user:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const getUserList = useCallback(async () => {
    try {
      console.log('Calling service /training/get_user_list with request:', {});

      const result = await callService(
        '/training/get_user_list',
        'physical_ai_interfaces/srv/GetUserList',
        {}
      );

      console.log('getUserList service response:', result);
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

        console.log('getDatasetList service response:', result);
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
      console.log('Calling service /training/get_policy_list with request:', {});

      const result = await callService(
        '/training/get_available_policy',
        'physical_ai_interfaces/srv/GetPolicyList',
        {}
      );

      console.log('getPolicyList service response:', result);
      return result;
    } catch (error) {
      console.error('Failed to get policy list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const getModelWeightList = useCallback(async () => {
    try {
      console.log('Calling service /training/get_model_weight_list with request:', {});

      const result = await callService(
        '/training/get_model_weight_list',
        'physical_ai_interfaces/srv/GetModelWeightList',
        {}
      );

      console.log('getModelWeightList service response:', result);
      return result;
    } catch (error) {
      console.error('Failed to get model weight list:', error);
      throw new Error(`${error.message || error}`);
    }
  }, [callService]);

  const sendTrainingCommand = useCallback(
    async (command) => {
      try {
        console.log('Calling service /training/send_training_command with request:', {
          command: command,
          training_info: trainingInfo,
        });

        let command_enum;
        switch (command) {
          case 'start':
            command_enum = TrainingCommand.START;
            break;
          case 'finish':
            command_enum = TrainingCommand.FINISH;
            break;
          default:
            throw new Error(`Unknown command: ${command}`);
        }

        const result = await callService(
          '/training/command',
          'physical_ai_interfaces/srv/SendTrainingCommand',
          {
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
          }
        );

        console.log('sendTrainingCommand service response:', result);
        return result;
      } catch (error) {
        console.error('Failed to send training command:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService, trainingInfo]
  );

  const sendEditDatasetCommand = useCallback(
    async (command) => {
      try {
        console.log('Calling service /training/send_edit_dataset_command with request:', {
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

        const result = await callService(
          '/dataset/edit',
          'physical_ai_interfaces/srv/SetDatasetEditInfo',
          {
            mode: command_enum,
            dataset_list: editDatasetInfo.datasetList,
            delete_dataset_path: editDatasetInfo.deleteDatasetPath,
            output_path: editDatasetInfo.outputPath,
            delete_episode_num: editDatasetInfo.deleteEpisodeNum,
            upload_huggingface: editDatasetInfo.uploadHuggingface,
          }
        );

        console.log('sendEditDatasetCommand service response:', result);
        return result;
      } catch (error) {
        console.error('Failed to send edit dataset command:', error);
        throw new Error(`${error.message || error}`);
      }
    },
    [callService, editDatasetInfo]
  );

  return {
    callService,
    sendRecordCommand,
    getImageTopicList,
    getRobotTypeList,
    setRobotType,
    registerHFUser,
    getRegisteredHFUser,
    getUserList,
    getDatasetList,
    getPolicyList,
    getModelWeightList,
    sendTrainingCommand,
    sendEditDatasetCommand,
  };
}
