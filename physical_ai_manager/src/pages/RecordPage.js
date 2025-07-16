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
import { MdKeyboardDoubleArrowLeft, MdKeyboardDoubleArrowRight, MdTask } from 'react-icons/md';
import ControlPanel from '../components/ControlPanel';
import HeartbeatStatus from '../components/HeartbeatStatus';
import ImageGrid from '../components/ImageGrid';
import InfoPanel from '../components/InfoPanel';
import { addTag } from '../features/tasks/taskSlice';
import { setIsFirstLoadFalse } from '../features/ui/uiSlice';

export default function RecordPage({ isActive = true }) {
  const dispatch = useDispatch();

  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const useMultiTaskMode = useSelector((state) => state.tasks.useMultiTaskMode);
  const multiTaskIndex = useSelector((state) => state.tasks.multiTaskIndex);

  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);

  const isFirstLoad = useSelector((state) => state.ui.isFirstLoad.record);

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
    dispatch(setIsFirstLoadFalse('record'));
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

  const classTaskInstructionContainer = clsx(
    'absolute',
    'bottom-1',
    'left-10',
    'w-[40%]',
    'z-30',
    'flex',
    'flex-row',
    'items-center',
    'bg-gradient-to-r',
    'from-green-50/70',
    'to-emerald-50/70',
    'backdrop-blur-xs',
    'rounded-xl',
    'px-4',
    'py-3',
    'shadow-lg',
    'border',
    'border-green-100/50',
    'hover:shadow-xl',
    'hover:from-green-50/80',
    'hover:to-emerald-50/80',
    'transition-all',
    'duration-300'
  );

  const classTaskIcon = clsx('text-green-600', 'text-2xl', 'mr-3', 'flex-shrink-0');

  const classTaskLabel = clsx(
    'text-green-700',
    'font-semibold',
    'text-lg',
    'mr-3',
    'flex-shrink-0'
  );

  const classTaskValue = clsx(
    'text-green-800',
    'text-lg',
    'font-medium',
    'flex-1',
    'min-w-0',
    'whitespace-normal'
  );

  return (
    <div className={classMainContainer}>
      <div className={classContentsArea}>
        <div className="w-full h-full flex flex-col relative">
          <div className={classRobotTypeContainer}>
            <div className={classRobotType}>Robot Type</div>
            <div className={classRobotTypeValue}>{taskStatus?.robotType}</div>
          </div>
          <div className={classHeartbeatStatus}>
            <HeartbeatStatus />
          </div>
          <div className={classImageGridContainer}>
            <ImageGrid isActive={isActive} />
            {useMultiTaskMode && (
              <div className={classTaskInstructionContainer}>
                <div className="flex flex-col">
                  <div className="flex flex-row">
                    <MdTask className={classTaskIcon} />
                    <span className={classTaskLabel}>Current Task</span>
                    {multiTaskIndex !== undefined && (
                      <span className={classTaskLabel}>
                        {`[${multiTaskIndex + 1} / ${taskInfo.taskInstruction.length}]`}
                      </span>
                    )}
                  </div>
                  <span className={classTaskValue}>{taskStatus.currentTaskInstruction}</span>
                </div>
              </div>
            )}
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
            <InfoPanel />
          </div>
        </div>
      </div>
      <ControlPanel />
    </div>
  );
}
