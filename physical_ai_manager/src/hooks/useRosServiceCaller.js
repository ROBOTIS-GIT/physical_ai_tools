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

import { useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import TaskCommand from '../constants/taskCommand';

export function useRosServiceCaller(rosbridgeUrl) {
  const rosRef = useRef(null);

  const getRosConnection = useCallback(() => {
    return new Promise((resolve, reject) => {
      if (rosRef.current && rosRef.current.isConnected === true) {
        resolve(rosRef.current);
        return;
      }

      // Create new ROS connection
      const ros = new ROSLIB.Ros({ url: rosbridgeUrl });
      const connectionTimeout = setTimeout(() => {
        reject(new Error('ROS connection timeout - rosbridge server is not running'));
      }, 2000); // 2 second timeout for faster feedback

      ros.on('connection', () => {
        clearTimeout(connectionTimeout);
        console.log('Connected to ROS bridge');
        rosRef.current = ros;
        resolve(ros);
      });

      ros.on('error', (error) => {
        clearTimeout(connectionTimeout);
        console.error('ROS connection error:', error);
        rosRef.current = null;
        reject(new Error(`ROS connection failed: ${error.message || error}`));
      });

      ros.on('close', () => {
        console.log('ROS connection closed');
        rosRef.current = null;
      });
    });
  }, [rosbridgeUrl]);

  const callService = useCallback(
    async (serviceName, serviceType, request) => {
      try {
        console.log(`Attempting to call service: ${serviceName}`);
        const ros = await getRosConnection();

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
    [getRosConnection]
  );

  const sendRecordCommand = useCallback(
    async (command, task_info) => {
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
          case 'rerecord':
            command_enum = TaskCommand.RERECORD;
            break;
          case 'finish':
            command_enum = TaskCommand.FINISH;
            break;
          default:
            throw new Error(`Unknown command: ${command}`);
        }

        // Validate required task_info fields
        if (!task_info) {
          throw new Error('Task info is required');
        }

        const request = {
          task_info: {
            task_name: String(task_info.taskName || ''),
            task_type: String(task_info.taskType || ''),
            user_id: String(task_info.userId || ''),
            task_instruction: String(task_info.taskInstruction || ''),
            policy_path: String(task_info.policyPath || ''),
            record_inference_mode: Boolean(task_info.recordInferenceMode),
            fps: Number(task_info.fps) || 0,
            tags: task_info.tags || [],
            warmup_time_s: Number(task_info.warmupTime) || 0,
            episode_time_s: Number(task_info.episodeTime) || 0,
            reset_time_s: Number(task_info.resetTime) || 0,
            num_episodes: Number(task_info.numEpisodes) || 0,
            push_to_hub: Boolean(task_info.pushToHub),
            private_mode: Boolean(task_info.privateMode),
            use_optimized_save_mode: Boolean(task_info.useOptimizedSave),
          },
          command: Number(command_enum),
        };

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
    [callService]
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

  return {
    callService,
    sendRecordCommand,
    getImageTopicList,
    getRobotTypeList,
    setRobotType,
    registerHFUser,
    getRegisteredHFUser,
  };
}
