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
          },
          (error) => {
            console.error('Service call failed:', error);
          }
        );
      } catch (error) {
        console.error('Failed to create ROS connection:', error);
      }
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
    (command, task_info) => {
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
        case 'terminate_all':
          command_enum = 6;
      }
      callService('recording/command', 'physical_ai_interfaces/srv/SendRecordingCommand', {
        command: Number(command_enum),
        task_name: String(task_info.taskName),
        robot_type: String(task_info.robotType),
        frequency: Number(task_info.fps),
        task_instruction: String(task_info.taskInstruction),
        repo_id: String(task_info.repoId),
        use_image_buffer: Boolean(task_info.useImageBuffer),
        reset_time: Number(task_info.resetTime),
        episode_time: Number(task_info.episodeTime),
        episode_num: Number(task_info.numEpisodes),
        push_to_hub: Boolean(task_info.pushToHub),
        private_mode: Boolean(task_info.privateMode),
      });
    },
    [callService]
  );

  return { callService, setGuiPage, sendRecordCommand };
}
