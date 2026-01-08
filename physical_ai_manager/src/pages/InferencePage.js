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

import React, { useState, useEffect, useRef } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import { MdKeyboardDoubleArrowLeft, MdKeyboardDoubleArrowRight } from 'react-icons/md';
import ControlPanel from '../components/ControlPanel';
import HeartbeatStatus from '../components/HeartbeatStatus';
import ImageGrid from '../components/ImageGrid';
import InferencePanel from '../components/InferencePanel';
import { addTag } from '../features/tasks/taskSlice';
import { setIsFirstLoadFalse } from '../features/ui/uiSlice';
import { setTaskInfo } from '../features/tasks/taskSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import ROSLIB from 'roslib';
import rosConnectionManager from '../utils/rosConnectionManager';

export default function InferencePage({ isActive = true }) {
  const dispatch = useDispatch();

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  const downloadProgressRef = useRef({ current: 0, total: 0, percentage: 0 });
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);

  useEffect(() => {
    let topic = null;
    const setupSubscription = async () => {
      try {
        const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
        if (ros && ros.isConnected) {
          topic = new ROSLIB.Topic({
            ros: ros,
            name: '/huggingface/status',
            messageType: 'physical_ai_interfaces/msg/HFOperationStatus'
          });
          topic.subscribe((message) => {
            if (message.operation === 'download' && message.status === 'in_progress') {
              downloadProgressRef.current = {
                current: message.progress_current || 0,
                total: message.progress_total || 0,
                percentage: message.progress_percentage || 0
              };
            }
          });
        }
      } catch (e) {
        console.error('Failed to subscribe to HF status:', e);
      }
    };
    setupSubscription();
    return () => {
      if (topic) topic.unsubscribe();
    };
  }, [rosbridgeUrl]);

  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const taskInfo = useSelector((state) => state.tasks.taskInfo);

  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);
  const [isDemoLoading, setIsDemoLoading] = useState(false);

  const isFirstLoad = useSelector((state) => state.ui.isFirstLoad.inference);

  // ROS service caller
  const { controlHfServer, setInferenceServerInfo, sendRecordCommand, browseFile } = useRosServiceCaller();

  // Demo mode handler
  const handleDemoMode = async () => {
    if (isDemoLoading) return;

    try {
      setIsDemoLoading(true);

      const demoConfig = {
        repoId: 'ROBOTIS/ffw_bg2_demo_model',
        policyPath: '/root/.cache/huggingface/ROBOTIS/ffw_bg2_demo_model/ffw_bg2_rev4_pick_coffee_bottle_env5_1_to_31_joint_fix_20k',
        taskInstruction: 'Place bottles in color-matching boxes: red→top left, green→bottom left, white→top right, orange→bottom right',
        fps: 15,
        robotType: 'ffw_bg2_rev4'
      };

      // 1. Update UI values first
      dispatch(setTaskInfo({
        ...taskInfo,
        taskInstruction: [demoConfig.taskInstruction],
        policyPath: demoConfig.policyPath,
        fps: demoConfig.fps
      }));

      // 2. Check if model exists and download if needed
      toast.loading('Checking demo model...', { id: 'demo-download' });

      // Check if model directory exists and download is complete
      let modelExists = false;
      try {
        const checkResult = await browseFile('browse', demoConfig.policyPath);
        console.log('Initial check result:', checkResult);

        if (checkResult && checkResult.success && checkResult.items) {
          const names = checkResult.items.map(i => i.name).join(', ');
          console.log('Items in folder:', names);

          // Check for incomplete/temp files that indicate download in progress
          const hasIncompleteFiles = checkResult.items.some(item =>
            item.name.endsWith('.incomplete') ||
            item.name.endsWith('.tmp') ||
            item.name.endsWith('.lock') ||
            item.name.startsWith('.') && item.name.includes('incomplete')
          );

          if (hasIncompleteFiles) {
            console.log('Download in progress: found incomplete/temp files');
            modelExists = false;
          } else {
            // Model exists if there are .safetensors files AND config.json (both required)
            const hasSafetensors = checkResult.items.some(item =>
              item.name.endsWith('.safetensors')
            );
            const hasConfig = checkResult.items.some(item =>
              item.name === 'config.json'
            );
            modelExists = hasSafetensors && hasConfig;
            console.log(`Model check: safetensors=${hasSafetensors}, config=${hasConfig}, exists=${modelExists}`);
          }
        }
      } catch (e) {
        console.log('Model check failed, will download:', e);
      }

      if (!modelExists) {
        // Start download
        toast.loading('Downloading demo model (this may take a few minutes)...', { id: 'demo-download' });
        const downloadResult = await controlHfServer('download', demoConfig.repoId, 'policy');

        if (!downloadResult || !downloadResult.success) {
          toast.error(downloadResult?.message || 'Failed to start download', { id: 'demo-download' });
          return;
        }

        // Poll for download completion
        let downloadComplete = false;
        let attempts = 0;
        // Removed maxAttempts limit as per user request to support long downloads

        while (!downloadComplete) {
          await new Promise(resolve => setTimeout(resolve, 5000)); // Wait 5 seconds

          try {
            const checkAgain = await browseFile('browse', demoConfig.policyPath);
            if (checkAgain && checkAgain.success && checkAgain.items) {
              // Debug logging
              const names = checkAgain.items.map(i => i.name).join(', ');
              console.log('Polling check:', names);

              // Check for incomplete/temp files that indicate download in progress
              const hasIncompleteFiles = checkAgain.items.some(item =>
                item.name.endsWith('.incomplete') ||
                item.name.endsWith('.tmp') ||
                item.name.endsWith('.lock') ||
                item.name.startsWith('.') && item.name.includes('incomplete')
              );

              if (!hasIncompleteFiles) {
                // Check for .safetensors files AND config.json (both required)
                const hasSafetensors = checkAgain.items.some(item =>
                  item.name.endsWith('.safetensors')
                );
                const hasConfig = checkAgain.items.some(item =>
                  item.name === 'config.json'
                );

                if (hasSafetensors && hasConfig) {
                  downloadComplete = true;
                  console.log('Download complete: found safetensors and config.json');
                  toast.success('Model download complete!', { id: 'demo-download' });
                } else {
                  console.log(`Waiting: safetensors=${hasSafetensors}, config=${hasConfig}`);
                }
              } else {
                console.log('Download still in progress: incomplete files found');
              }
            }
          } catch (e) {
            console.log('Polling error:', e);
            // Continue polling
          }

          if (!downloadComplete) {
            if (downloadProgressRef.current.total > 0) {
              const currentGB = (downloadProgressRef.current.current / (1024 * 1024 * 1024)).toFixed(2);
              const totalGB = (downloadProgressRef.current.total / (1024 * 1024 * 1024)).toFixed(2);
              const percent = (downloadProgressRef.current.percentage * 100).toFixed(1);
              toast.loading(`Downloading: ${currentGB}/${totalGB} GB (${percent}%)`, { id: 'demo-download' });
            } else {
              attempts++;
              toast.loading(`Downloading model... (${attempts * 5}s)`, { id: 'demo-download' });
            }
          }
        }
      }

      toast.success('Model ready!', { id: 'demo-download' });

      // 3. Configure inference server
      toast.loading('Configuring server...', { id: 'demo-config' });
      const configResult = await setInferenceServerInfo({
        server_ip: '0.0.0.0',
        server_port: 5555,
        policy_type: 'GR00T_N1_5',
        policy_path: demoConfig.policyPath,
        robot_type: demoConfig.robotType
      });

      if (!configResult || !configResult.success) {
        toast.error(configResult?.message || 'Failed to configure server', { id: 'demo-config' });
        return;
      }
      toast.success('Server configured!', { id: 'demo-config' });

      // 4. Start inference
      toast.loading('Starting inference...', { id: 'demo-start' });
      const startResult = await sendRecordCommand('start_inference');

      if (!startResult || !startResult.success) {
        toast.error(startResult?.message || 'Failed to start inference', { id: 'demo-start' });
        return;
      }

      toast.success('🎮 Demo mode started!', { id: 'demo-start' });

    } catch (error) {
      console.error('Demo mode error:', error);
      toast.error(`Demo mode failed: ${error.message}`);
    } finally {
      setIsDemoLoading(false);
    }
  };

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  useEffect(() => {
    if (isFirstLoad && taskStatus.robotType !== '' && taskInfo.tags.length === 0) {
      dispatch(addTag(taskStatus.robotType));
      dispatch(addTag('robotis'));
    }
    dispatch(setIsFirstLoadFalse('inference'));
  }, [taskInfo.tags, taskStatus.robotType, dispatch, isFirstLoad]);

  const classMainContainer = 'h-full flex flex-col overflow-hidden';
  const classContentsArea = 'flex-1 flex min-h-0 pt-0 px-0 justify-center items-start';
  const classImageGridContainer = clsx(
    'transition-all',
    'duration-300',
    'ease-in-out',
    'flex',
    'items-center',
    'justify-center',
    'min-h-0',
    'h-full',
    'overflow-hidden',
    'm-2',
    {
      'flex-[12]': isRightPanelCollapsed,
      'flex-[10]': !isRightPanelCollapsed,
    }
  );

  const classRightPanelArea = clsx(
    'h-full',
    'w-full',
    'transition-all',
    'duration-300',
    'ease-in-out',
    'relative',
    'overflow-y-auto',
    {
      'flex-[0_0_40px]': isRightPanelCollapsed,
      'flex-[1]': !isRightPanelCollapsed,
      'min-w-[60px]': isRightPanelCollapsed,
      'min-w-[400px]': !isRightPanelCollapsed,
      'max-w-[60px]': isRightPanelCollapsed,
      'max-w-[400px]': !isRightPanelCollapsed,
    }
  );

  const classHideButton = clsx(
    'absolute',
    'top-3',
    'bg-white',
    'border',
    'border-gray-300',
    'rounded-full',
    'w-12',
    'h-12',
    'flex',
    'items-center',
    'justify-center',
    'shadow-md',
    'hover:bg-gray-50',
    'transition-all',
    'duration-200',
    'z-10',
    {
      'left-2': isRightPanelCollapsed,
      'left-[10px]': !isRightPanelCollapsed,
    }
  );

  const classRightPanel = clsx(
    'h-full',
    'flex',
    'flex-col',
    'items-center',
    'overflow-hidden',
    'transition-opacity',
    'duration-300',
    {
      'opacity-0': isRightPanelCollapsed,
      'opacity-100': !isRightPanelCollapsed,
      'pointer-events-none': isRightPanelCollapsed,
      'pointer-events-auto': !isRightPanelCollapsed,
    }
  );

  const classRobotTypeContainer = clsx(
    'absolute',
    'top-4',
    'left-4',
    'z-20',
    'flex',
    'flex-row',
    'items-center',
    'bg-white/90',
    'backdrop-blur-sm',
    'rounded-full',
    'px-3',
    'py-1',
    'shadow-md',
    'border',
    'border-gray-100'
  );
  const classRobotType = clsx('ml-2 mr-1 my-2 text-gray-600 text-lg');
  const classRobotTypeValue = clsx(
    'mx-1 my-2 px-2 text-lg text-blue-600 focus:outline-none bg-blue-100 rounded-full'
  );

  const classHeartbeatStatus = clsx('absolute', 'top-20', 'left-5', 'z-10');

  return (
    <div className={classMainContainer}>
      <div className={classContentsArea}>
        <div className="w-full h-full flex flex-col relative">
          <div className={classRobotTypeContainer}>
            <div className={classRobotType}>Robot Type</div>
            <div className={classRobotTypeValue}>{taskStatus?.robotType}</div>
            {taskStatus?.robotType === 'ffw_bg2_rev4' && (
              <button
                onClick={handleDemoMode}
                disabled={isDemoLoading}
                className={clsx(
                  'ml-2',
                  'px-3',
                  'py-1',
                  'text-sm',
                  'font-medium',
                  'bg-gradient-to-r',
                  'from-purple-500',
                  'to-indigo-600',
                  'text-white',
                  'rounded-full',
                  'hover:from-purple-600',
                  'hover:to-indigo-700',
                  'transition-all',
                  'duration-200',
                  'shadow-md',
                  'hover:shadow-lg',
                  'transform',
                  'hover:scale-105',
                  'disabled:opacity-50',
                  'disabled:cursor-not-allowed',
                  'disabled:transform-none'
                )}
                title="Start Demo Mode"
              >
                {isDemoLoading ? '⏳ Loading...' : '🎮 Demo'}
              </button>
            )}
          </div>
          <div className={classHeartbeatStatus}>
            <HeartbeatStatus />
          </div>
          <div className={classImageGridContainer}>
            <ImageGrid isActive={isActive} />
          </div>
        </div>
        <div className={classRightPanelArea}>
          <button
            onClick={() => setIsRightPanelCollapsed(!isRightPanelCollapsed)}
            className={classHideButton}
            title="Hide"
          >
            <span className="text-gray-600 text-3xl transition-transform duration-200">
              {isRightPanelCollapsed ? (
                <MdKeyboardDoubleArrowLeft />
              ) : (
                <MdKeyboardDoubleArrowRight />
              )}
            </span>
          </button>
          <div className={classRightPanel}>
            <div className="w-full min-h-10"></div>
            <InferencePanel />
          </div>
        </div>
      </div>
      <ControlPanel />
    </div>
  );
}
