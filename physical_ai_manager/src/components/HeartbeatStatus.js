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

import React, { useRef, useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import { setHeartbeatStatus } from '../features/tasks/taskSlice';

/**
 * HeartbeatStatus Component
 *
 * Component that displays connection status by receiving heartbeat signals through ROS topics
 * Monitors system connection status by periodically receiving std_msgs/Empty messages
 */
export default function HeartbeatStatus({
  timeoutMs = 3000,
  disconnectTimeoutMs = 10000,
  className = '',
  showLabel = true,
  size = 'medium',
}) {
  const dispatch = useDispatch();
  const heartbeatStatus = useSelector((state) => state.tasks.heartbeatStatus);
  const lastHeartbeatTime = useSelector((state) => state.tasks.lastHeartbeatTime);

  const intervalRef = useRef(null);

  // Heartbeat status: 'connected', 'timeout', 'disconnected'
  const getStatusInfo = () => {
    switch (heartbeatStatus) {
      case 'connected':
        return {
          dotColor: 'bg-green-500',
          color: 'text-green-500',
          bgColor: 'bg-green-100',
          borderColor: 'border-green-300',
          label: 'Connected',
          description: `Last heartbeat: ${
            lastHeartbeatTime ? new Date(lastHeartbeatTime).toLocaleTimeString() : 'Never'
          }`,
        };
      case 'timeout':
        return {
          dotColor: 'bg-yellow-500',
          color: 'text-yellow-500',
          bgColor: 'bg-yellow-100',
          borderColor: 'border-yellow-300',
          label: 'Timeout',
          description: `No heartbeat for ${timeoutMs}ms`,
        };
      case 'disconnected':
      default:
        return {
          dotColor: 'bg-red-500',
          color: 'text-red-500',
          bgColor: 'bg-red-100',
          borderColor: 'border-red-300',
          label: 'Disconnected',
          description: 'ROS connection not available',
        };
    }
  };

  // Set up interval for status checking
  useEffect(() => {
    // Clear existing interval
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }

    // Check status every 1 second
    intervalRef.current = setInterval(() => {
      const now = Date.now();

      if (!lastHeartbeatTime) {
        // If heartbeat has not been received yet
        if (heartbeatStatus !== 'disconnected') {
          dispatch(setHeartbeatStatus('disconnected'));
        }
        return;
      }

      const timeSinceLastHeartbeat = now - lastHeartbeatTime;

      if (timeSinceLastHeartbeat >= disconnectTimeoutMs) {
        // If 10 seconds have passed - disconnected
        if (heartbeatStatus !== 'disconnected') {
          dispatch(setHeartbeatStatus('disconnected'));
        }
      } else if (timeSinceLastHeartbeat >= timeoutMs) {
        // If 3 seconds have passed - timeout
        if (heartbeatStatus !== 'timeout') {
          dispatch(setHeartbeatStatus('timeout'));
        }
      } else {
        // If 3 seconds have passed - connected
        if (heartbeatStatus !== 'connected') {
          dispatch(setHeartbeatStatus('connected'));
        }
      }
    }, 1000); // Check every 1 second

    // Cleanup function
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [lastHeartbeatTime, timeoutMs, disconnectTimeoutMs, heartbeatStatus, dispatch]);

  const statusInfo = getStatusInfo();

  // Size-based styles
  const sizeClasses = {
    small: {
      container: 'px-2 py-1',
      dot: 'w-2 h-2',
      text: 'text-xs',
    },
    medium: {
      container: 'px-3 py-2',
      dot: 'w-3 h-3',
      text: 'text-sm',
    },
    large: {
      container: 'px-4 py-3',
      dot: 'w-4 h-4',
      text: 'text-base',
    },
  };

  const currentSize = sizeClasses[size] || sizeClasses.medium;

  const containerClasses = clsx(
    'inline-flex',
    'items-center',
    'gap-2',
    'rounded-full',
    'border',
    'transition-all',
    'duration-200',
    'select-none',
    'shadow-md',
    currentSize.container,
    statusInfo.bgColor,
    statusInfo.borderColor,
    className
  );

  const dotClasses = clsx(
    'rounded-full',
    'transition-colors',
    'duration-200',
    currentSize.dot,
    statusInfo.dotColor
  );

  const textClasses = clsx('font-medium', 'whitespace-nowrap', currentSize.text, statusInfo.color);

  return (
    <div className={containerClasses}>
      <div className={dotClasses} />
      {showLabel && <span className={textClasses}>{statusInfo.label}</span>}
    </div>
  );
}
