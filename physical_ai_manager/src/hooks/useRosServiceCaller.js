import { useRef, useCallback } from 'react';
import ROSLIB from 'roslib';

export function useRosServiceCaller(rosbridgeUrl) {
  const rosRef = useRef(null);

  const getRosConnection = useCallback(() => {
    if (!rosRef.current || rosRef.current.isConnected === false) {
      rosRef.current = new ROSLIB.Ros({ url: rosbridgeUrl });

      rosRef.current.on('connection', () => {
        console.log('Connected to ROS bridge');
      });

      rosRef.current.on('error', (error) => {
        console.error('ROS connection error:', error);
        rosRef.current = null;
      });

      rosRef.current.on('close', () => {
        console.log('ROS connection closed');
        rosRef.current = null;
      });
    }
    return rosRef.current;
  }, [rosbridgeUrl]);

  const callService = useCallback(
    (serviceName, serviceType, request) => {
      return new Promise((resolve, reject) => {
        try {
          const ros = getRosConnection();

          const service = new ROSLIB.Service({
            ros,
            name: serviceName,
            serviceType: serviceType,
          });
          const req = new ROSLIB.ServiceRequest(request);

          service.callService(
            req,
            (result) => {
              console.log('Service call successful:', result);
              resolve(result);
            },
            (error) => {
              console.error('Service call failed:', error);
              reject(error);
            }
          );
        } catch (error) {
          console.error('Failed to create ROS connection:', error);
          reject(error);
        }
      });
    },
    [getRosConnection]
  );

  const setGuiPage = useCallback(
    (pageName, option) => {
      console.log('setGuiPage called with:', pageName, option);
      try {
        callService('/gui/set_page', 'gaemi_interfaces/srv/SetGUIPage', {
          page_name: pageName,
          option: option,
        });
      } catch (error) {
        console.error('setGuiPage failed:', error);
      }
    },
    [callService]
  );

  const sendRecordCommand = useCallback(
    async (command, task_info, model_path = '') => {
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
      }

      return await callService('/task/command', 'physical_ai_interfaces/srv/SendCommand', {
        task_info: {
          task_name: String(task_info.taskName),
          robot_type: String(task_info.robotType),
          task_type: String(task_info.taskType),
          repo_id: String(task_info.repoId),
          task_instruction: String(task_info.taskInstruction),
          fps: Number(task_info.fps),
          tags: task_info.tags,
          warmup_time_s: Number(task_info.warmupTime),
          episode_time_s: Number(task_info.episodeTime),
          reset_time_s: Number(task_info.resetTime),
          num_episodes: Number(task_info.numEpisodes),
          resume: Boolean(task_info.resume),
          push_to_hub: Boolean(task_info.pushToHub),
          private_mode: Boolean(task_info.privateMode),
          use_image_buffer: Boolean(task_info.useImageBuffer),
        },
        command: Number(command_enum),
        model_path: String(model_path),
      });
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
