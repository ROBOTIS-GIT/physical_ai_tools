import { useRef, useCallback } from 'react';
import ROSLIB from 'roslib';

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

  const setGuiPage = useCallback(
    async (pageName, option) => {
      console.log('setGuiPage called with:', pageName, option);
      try {
        await callService('/gui/set_page', 'gaemi_interfaces/srv/SetGUIPage', {
          page_name: pageName,
          option: option,
        });
      } catch (error) {
        console.error('setGuiPage failed:', error);
        throw error;
      }
    },
    [callService]
  );

  const sendRecordCommand = useCallback(
    async (command, task_info, model_path = '') => {
      try {
        let command_enum;
        switch (command) {
          case 'none':
            command_enum = 0;
            break;
          case 'start_record':
            command_enum = 1;
            break;
          case 'start_inference':
            command_enum = 2;
            break;
          case 'stop':
            command_enum = 3;
            break;
          case 'next':
            command_enum = 4;
            break;
          case 'rerecord':
            command_enum = 5;
            break;
          case 'finish':
            command_enum = 6;
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
            robot_type: String(task_info.robotType || ''),
            task_type: String(task_info.taskType || ''),
            repo_id: String(task_info.repoId || ''),
            task_instruction: String(task_info.taskInstruction || ''),
            fps: Number(task_info.fps) || 0,
            tags: task_info.tags || [],
            warmup_time_s: Number(task_info.warmupTime) || 0,
            episode_time_s: Number(task_info.episodeTime) || 0,
            reset_time_s: Number(task_info.resetTime) || 0,
            num_episodes: Number(task_info.numEpisodes) || 0,
            resume: Boolean(task_info.resume),
            push_to_hub: Boolean(task_info.pushToHub),
            private_mode: Boolean(task_info.privateMode),
            use_image_buffer: Boolean(task_info.useImageBuffer),
          },
          command: Number(command_enum),
          model_path: String(model_path),
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
        '/get_image_topic_list',
        'physical_ai_interfaces/srv/GetImageTopicList',
        {}
      );
      return result;
    } catch (error) {
      console.error('Failed to get image topic list:', error);
      throw error;
    }
  }, [callService]);

  return { callService, setGuiPage, sendRecordCommand, getImageTopicList };
}
