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
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast, { useToasterStore } from 'react-hot-toast';
import { MdKeyboardDoubleArrowLeft, MdKeyboardDoubleArrowRight } from 'react-icons/md';

import RecordControlPanel from '../components/RecordControlPanel';
import HeartbeatStatus from '../components/HeartbeatStatus';
import InlineSystemStatus from '../components/InlineSystemStatus';
import ImageGrid, { RECORD_LAYOUT } from '../components/ImageGrid';
import SegmentPanel from '../components/SegmentPanel';
import RecordTopicMonitor from '../components/RecordTopicMonitor';
import { setIsFirstLoadFalse } from '../features/ui/uiSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

export default function RecordPage({ isActive = true }) {
  const dispatch = useDispatch();
  const { sendRecordCommand } = useRosServiceCaller();

  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const joystickMode = useSelector((state) => state.tasks.joystickMode);

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);
  const [isMonitorPanelCollapsed, setIsMonitorPanelCollapsed] = useState(true);

  const isFirstLoad = useSelector((state) => state.ui.isFirstLoad.record);

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  useEffect(() => {
    dispatch(setIsFirstLoadFalse('record'));
  }, [dispatch, isFirstLoad]);

  // Refresh topic subscriptions when entering the page so the topic
  // monitor picks up topics that appeared after robot-type setup.
  useEffect(() => {
    if (isActive) {
      sendRecordCommand('refresh_topics').catch(() => {});
    }
  }, [isActive, sendRecordCommand]);

  const classMainContainer = 'h-full flex flex-col overflow-hidden';
  const classContentsArea = 'flex-1 flex min-h-0 pt-0 px-0 justify-center items-start';
  const classLeftArea = clsx(
    'transition-all',
    'duration-300',
    'ease-in-out',
    'flex',
    'flex-col',
    'min-h-0',
    'h-full',
    'overflow-hidden',
    'm-2',
    {
      'flex-[12]': isRightPanelCollapsed && isMonitorPanelCollapsed,
      'flex-[10]': (isRightPanelCollapsed && !isMonitorPanelCollapsed) ||
                   (!isRightPanelCollapsed && isMonitorPanelCollapsed),
      'flex-[8]': !isRightPanelCollapsed && !isMonitorPanelCollapsed,
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
    'min-h-full',
    'flex',
    'flex-col',
    'items-center',
    'pb-4',
    'transition-opacity',
    'duration-300',
    {
      'opacity-0': isRightPanelCollapsed,
      'opacity-100': !isRightPanelCollapsed,
      'pointer-events-none': isRightPanelCollapsed,
      'pointer-events-auto': !isRightPanelCollapsed,
    }
  );

  const classMonitorPanelArea = clsx(
    'h-full',
    'w-full',
    'transition-all',
    'duration-300',
    'ease-in-out',
    'relative',
    'overflow-y-auto',
    {
      'flex-[0_0_40px]': isMonitorPanelCollapsed,
      'flex-[1]': !isMonitorPanelCollapsed,
      'min-w-[60px]': isMonitorPanelCollapsed,
      'min-w-[350px]': !isMonitorPanelCollapsed,
      'max-w-[60px]': isMonitorPanelCollapsed,
      'max-w-[350px]': !isMonitorPanelCollapsed,
    }
  );

  const classMonitorHideButton = clsx(
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
      'left-2': isMonitorPanelCollapsed,
      'left-[10px]': !isMonitorPanelCollapsed,
    }
  );

  const classMonitorPanel = clsx(
    'min-h-full',
    'w-full',
    'flex',
    'flex-col',
    'items-center',
    'pb-4',
    'transition-opacity',
    'duration-300',
    {
      'opacity-0': isMonitorPanelCollapsed,
      'opacity-100': !isMonitorPanelCollapsed,
      'pointer-events-none': isMonitorPanelCollapsed,
      'pointer-events-auto': !isMonitorPanelCollapsed,
    }
  );

  const classTopBar = clsx(
    'absolute', 'top-4', 'left-4', 'right-4', 'z-20',
    'flex', 'items-center', 'gap-4'
  );
  const classRobotTypeContainer = clsx(
    'flex', 'flex-row', 'items-center',
    'bg-white/90', 'backdrop-blur-sm',
    'rounded-full', 'px-3', 'py-1',
    'shadow-md', 'border', 'border-gray-100',
    'whitespace-nowrap', 'shrink-0'
  );
  const classRobotType = clsx('ml-1 mr-1 text-gray-600 text-sm');
  const classRobotTypeValue = clsx(
    'mx-0.5 px-2 py-0.5 text-sm text-blue-600 bg-blue-100 rounded-full',
    'whitespace-nowrap'
  );

  return (
    <div className={classMainContainer}>
      <div className={classContentsArea}>
        <div className={classLeftArea}>
          <div className="relative flex-1 min-h-0 overflow-hidden pt-20">
            <div className={classTopBar}>
              <div className={classRobotTypeContainer}>
                <div className={classRobotType}>Robot Type</div>
                <div className={classRobotTypeValue}>{taskStatus?.robotType}</div>
              </div>
              {joystickMode && (
                <div className={classRobotTypeContainer}>
                  <div className={classRobotType}>Mode</div>
                  <div className={classRobotTypeValue}>
                    {joystickMode}
                  </div>
                </div>
              )}
              <InlineSystemStatus />
              <HeartbeatStatus />
              <div className="flex-grow" />
              <RecordControlPanel />
            </div>
            <ImageGrid isActive={isActive} layout={RECORD_LAYOUT} />
          </div>
        </div>
        <div className={classMonitorPanelArea}>
          <button
            onClick={() => setIsMonitorPanelCollapsed(!isMonitorPanelCollapsed)}
            className={classMonitorHideButton}
            title="Topic Monitor"
          >
            <span className="text-gray-600 text-3xl transition-transform duration-200">
              {isMonitorPanelCollapsed ? (
                <MdKeyboardDoubleArrowLeft />
              ) : (
                <MdKeyboardDoubleArrowRight />
              )}
            </span>
          </button>
          <div className={classMonitorPanel}>
            <div className="w-full min-h-10"></div>
            <RecordTopicMonitor />
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
            <SegmentPanel />
          </div>
        </div>
      </div>
    </div>
  );
}
