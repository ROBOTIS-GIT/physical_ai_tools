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
import { MdKeyboardDoubleArrowLeft, MdKeyboardDoubleArrowRight, MdViewInAr } from 'react-icons/md';

import RecordControlPanel from '../components/RecordControlPanel';
import HeartbeatStatus from '../components/HeartbeatStatus';
import InlineSystemStatus from '../components/InlineSystemStatus';
import ImageGrid from '../components/ImageGrid';
import RobotViewer3D from '../components/RobotViewer3D';
import SegmentPanel from '../components/SegmentPanel';
import RecordTopicMonitor from '../components/RecordTopicMonitor';
import { setIsFirstLoadFalse } from '../features/ui/uiSlice';

export default function RecordPage({ isActive = true }) {
  const dispatch = useDispatch();

  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const joystickMode = useSelector((state) => state.tasks.joystickMode);

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);
  const [show3DViewer, setShow3DViewer] = useState(true);

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

  const classHeartbeatStatus = clsx('absolute', 'top-[4.5rem]', 'left-5', 'z-10');

  return (
    <div className={classMainContainer}>
      <div className={classContentsArea}>
        <div className={classLeftArea}>
          <div className="relative flex-[5] min-h-0 overflow-hidden pt-20">
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
              <div className="flex-grow" />
              <RecordControlPanel />
            </div>
            <div className={classHeartbeatStatus}>
              <HeartbeatStatus />
            </div>
            <ImageGrid isActive={isActive} />
          </div>
          <div className="flex-[4] min-h-[120px] flex flex-row items-center justify-center mx-1 gap-2 h-full relative">
            {show3DViewer && (
              <div className="h-[85%] rounded-2xl overflow-hidden relative" style={{ aspectRatio: '4/3' }}>
                <RobotViewer3D mode="live" />
              </div>
            )}
            <div className="h-[85%]" style={{ aspectRatio: '4/3' }}>
              <RecordTopicMonitor />
            </div>
            <button
              onClick={() => setShow3DViewer(!show3DViewer)}
              className={clsx(
                'absolute top-2 left-2 z-10 flex items-center gap-1.5 px-3 py-1.5 rounded-full text-sm font-medium transition-colors shadow-md border',
                show3DViewer
                  ? 'bg-indigo-500/90 text-white border-indigo-400 backdrop-blur-sm'
                  : 'bg-white/90 text-gray-600 border-gray-100 backdrop-blur-sm hover:bg-gray-50'
              )}
            >
              <MdViewInAr size={18} />
              3D
            </button>
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
