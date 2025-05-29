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
      callService('recording/command', 'physical_ai_interfaces/srv/SendRecordingCommand', {
        command: command,
        task_name: task_info.taskName,
        robot_type: task_info.robotType,
        frequency: task_info.fps,
        task_instruction: task_info.taskInstruction,
        repo_id: task_info.repoId,
        use_image_buffer: true,
        reset_time: task_info.resetTime,
        episode_time: task_info.episodeTime,
        episode_num: task_info.numEpisodes,
        push_to_hub: task_info.pushToHub,
        private_mode: false,
      });
    },
    [callService]
  );

  return { callService, setGuiPage, sendRecordCommand };
}
