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

import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { MdKeyboardDoubleArrowLeft, MdKeyboardDoubleArrowRight } from 'react-icons/md';
import toast, { useToasterStore } from 'react-hot-toast';
import ImageGrid from '../components/ImageGrid';
import ControlPanel from '../components/ControlPanel';
import InfoPanel from '../components/InfoPanel';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { useRosTaskStatus } from '../hooks/useRosTaskStatus';

export default function HomePage({ topics, setTopics, rosHost }) {
  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  // Subscribe to task status (includes task info) from ROS topic
  const {
    taskStatus,
    taskInfo,
    connected: taskStatusConnected,
    resetTaskToIdle,
  } = useRosTaskStatus(rosbridgeUrl, '/task/status');

  // Start with default values and update with data from topic
  const [info, setInfo] = useState(taskInfo);

  // Update info state when taskInfo changes
  useEffect(() => {
    if (taskInfo) {
      setInfo(taskInfo);
    }
  }, [taskInfo]);

  const [episodeStatus, setEpisodeStatus] = useState(taskStatus);

  // Update episodeStatus when taskStatus changes
  useEffect(() => {
    if (taskStatus) {
      setEpisodeStatus(taskStatus);
    }
  }, [taskStatus]);

  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);

  const { sendRecordCommand } = useRosServiceCaller(rosbridgeUrl);

  const handleControlCommand = async (cmd) => {
    console.log('Control command received:', cmd);
    let result;

    try {
      // Execute the appropriate command
      if (cmd === 'Start') {
        result = await sendRecordCommand('start_record', info);
      } else if (cmd === 'Stop') {
        result = await sendRecordCommand('stop', info);
      } else if (cmd === 'Retry') {
        result = await sendRecordCommand('rerecord', info);
      } else if (cmd === 'Next') {
        result = await sendRecordCommand('next', info);
      } else if (cmd === 'Finish') {
        result = await sendRecordCommand('finish', info);
      } else {
        console.warn(`Unknown command: ${cmd}`);
        toast.warning(`Unknown command: ${cmd}`);
        return;
      }

      console.log('Service call result:', result);

      // Handle service response
      if (result && result.success === false) {
        toast.error(`Command failed: ${result.message || 'Unknown error'}`);
        console.error(`Command '${cmd}' failed:`, result.message);
      } else if (result && result.success === true) {
        toast.success(`Command [${cmd}] executed successfully`);
        console.log(`Command '${cmd}' executed successfully`);

        // Reset task status to idle for Stop and Finish commands
        if (cmd === 'Stop' || cmd === 'Finish') {
          resetTaskToIdle();
        }
      } else {
        // Handle case where result is undefined or doesn't have success field
        console.warn(`Unexpected result format for command '${cmd}':`, result);
        toast.warning(`Command [${cmd}] completed with uncertain status`);
      }
    } catch (error) {
      console.error('Error handling control command:', error);

      // Show more specific error messages
      let errorMessage = error.message || error.toString();
      if (
        errorMessage.includes('ROS connection failed') ||
        errorMessage.includes('ROS connection timeout') ||
        errorMessage.includes('WebSocket')
      ) {
        toast.error(`🔌 ROS connection failed: rosbridge server is not running (${rosHost})`);
      } else if (errorMessage.includes('timeout')) {
        toast.error(`⏰ Command execution timeout [${cmd}]: Server did not respond`);
      } else {
        toast.error(`❌ Command execution failed [${cmd}]: ${errorMessage}`);
      }

      // Continue execution even after error - don't block UI
      console.log(`Continuing after error in command '${cmd}'`);
    }
  };

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
    'overflow-scroll',
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
    'overflow-scroll',
    {
      'opacity-0': isRightPanelCollapsed,
      'opacity-100': !isRightPanelCollapsed,
      'pointer-events-none': isRightPanelCollapsed,
      'pointer-events-auto': !isRightPanelCollapsed,
    }
  );

  return (
    <div className={classMainContainer}>
      <div className={classContentsArea}>
        <div className={classImageGridContainer}>
          <ImageGrid topics={topics} setTopics={setTopics} rosHost={rosHost} />
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
            <InfoPanel info={info} onChange={setInfo} disabled={taskStatus?.phase !== 0} />
          </div>
        </div>
      </div>
      <ControlPanel
        onCommand={handleControlCommand}
        episodeStatus={episodeStatus}
        taskInfo={taskInfo}
      />
    </div>
  );
}
