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

import React, { useState, useEffect, useRef, useCallback } from 'react';
import clsx from 'clsx';
import ProgressBar from './ProgressBar';
import { MdPlayArrow, MdStop, MdReplay, MdSkipNext, MdCheck } from 'react-icons/md';
import CompactStorageStatus from './CompactStorageStatus';
import StorageStatus from './StorageStatus';

const buttons = [
  { label: 'Start', icon: MdPlayArrow, color: '#1976d2' },
  { label: 'Stop', icon: MdStop, color: '#d32f2f' },
  { label: 'Retry', icon: MdReplay, color: '#fbc02d' },
  { label: 'Next', icon: MdSkipNext, color: '#388e3c' },
  { label: 'Finish', icon: MdCheck, color: '#388e3c' },
];

const phaseGuideMessages = {
  0: '📍 Waiting to start',
  1: '🔥 Warmup in progress',
  2: '🏠 Reset in progress',
  3: '🔴 Recording in progress',
  4: '◼️ Task Stopped',
  5: '⚡ Inference in progress',
};

export default function ControlPanel({ onCommand, episodeStatus }) {
  const icon_size = 70;
  const [hovered, setHovered] = useState(null);
  const [pressed, setPressed] = useState(null);
  const [started, setStarted] = useState(false);
  const startedRef = useRef(started);

  useEffect(() => {
    startedRef.current = started;
  }, [started]);

  // Check if button should be enabled based on phase
  const isButtonEnabled = useCallback(
    (label) => {
      const phase = episodeStatus?.phase;

      let isNone = phase == 0;
      let isStopped = phase == 4;

      switch (label) {
        case 'Start':
          // Start button disabled when task is running or when running flag is true
          return (isNone || isStopped) && !episodeStatus?.running;
        case 'Stop':
          // Stop button enabled only when task is running
          return !(isNone || isStopped);
        case 'Retry':
          // Retry button enabled only when task is stopped
          return !isNone;
        case 'Next':
          // Next button enabled only when task is stopped
          return !isNone;
        case 'Finish':
          // Finish button enabled only when task is stopped
          return !isNone;
        default:
          return false;
      }
    },
    [episodeStatus]
  );

  const handleCommand = useCallback(
    (label) => {
      if (onCommand) onCommand(label);
      console.log(label + ' command executed');
      if (label === 'Start') setStarted(true);
      if (label === 'Stop') setStarted(false);
    },
    [onCommand]
  );

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.key === 'ArrowLeft' && isButtonEnabled('Retry')) {
        handleCommand('Retry');
      } else if (e.key === 'ArrowRight' && isButtonEnabled('Next')) {
        handleCommand('Next');
      } else if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (!startedRef.current && isButtonEnabled('Start')) {
          handleCommand('Start');
        } else if (startedRef.current && isButtonEnabled('Stop')) {
          handleCommand('Stop');
        }
      } else if (
        (e.ctrlKey || e.metaKey) &&
        (e.key === 'c' || e.key === 'C') &&
        isButtonEnabled('Finish')
      ) {
        handleCommand('Finish');
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleCommand, !isButtonEnabled]);

  const classControlPanelBody = clsx(
    'h-56',
    'bg-gray-300',
    'rounded-3xl',
    'mx-8',
    'mt-2',
    'mb-4',
    'p-4',
    'flex',
    'flex-row',
    'items-center',
    'gap-2',
    'shadow-lg'
  );

  const classControlPanelButtons = (label, isDisabled) =>
    clsx(
      'text-4xl',
      'font-extrabold',
      'h-full',
      'flex-grow',
      'min-w-16',
      'rounded-2xl',
      'border-none',
      'cursor-pointer',
      'mr-2',
      'flex',
      'items-center',
      'justify-center',
      'flex-col',
      'gap-1',
      'bg-gray-100',
      'transition-all',
      'duration-200',
      {
        'bg-gray-300': pressed === label && !isDisabled,
        'bg-gray-200': hovered === label && pressed !== label && !isDisabled,
        'opacity-30': isDisabled,
        'cursor-not-allowed': isDisabled,
        'bg-gray-50': isDisabled,
      }
    );

  const classControlPanelButtonIcon = clsx(
    'bg-transparent',
    'rounded-full',
    'w-20',
    'h-20',
    'flex',
    'items-center',
    'justify-center',
    'mb-1'
  );

  const handleButtonKeyDown = (e, label, isDisabled) => {
    if (isDisabled) return;
    if (e.key === 'Enter') {
      handleCommand(label);
    }
    if (e.key === ' ') {
      e.preventDefault();
    }
  };

  const handleMouseEnter = (label, isDisabled) => {
    if (!isDisabled) {
      setHovered(label);
    }
  };

  const handleMouseLeave = () => {
    setHovered(null);
    setPressed(null);
  };

  const handleMouseDown = (label, isDisabled) => {
    if (!isDisabled) {
      setPressed(label);
    }
  };

  const handleMouseUp = () => {
    setPressed(null);
  };

  return (
    <div className={classControlPanelBody}>
      <div className="flex flex-[2] items-center w-full h-full gap-4">
        {buttons.map(({ label, icon: Icon, color }) => {
          const isDisabled = !isButtonEnabled(label);
          return (
            <button
              key={label}
              className={classControlPanelButtons(label, isDisabled)}
              style={{ fontFamily: 'Pretendard Variable' }}
              tabIndex={isDisabled ? -1 : 0}
              onClick={() => !isDisabled && handleCommand(label)}
              onKeyDown={(e) => handleButtonKeyDown(e, label, isDisabled)}
              onMouseEnter={() => handleMouseEnter(label, isDisabled)}
              onMouseLeave={handleMouseLeave}
              onMouseDown={() => handleMouseDown(label, isDisabled)}
              onMouseUp={handleMouseUp}
              disabled={isDisabled}
            >
              <span className={classControlPanelButtonIcon}>
                <Icon size={icon_size} color={isDisabled ? '#9ca3af' : color} />
              </span>
              {label}
            </button>
          );
        })}
      </div>
      <div className="flex flex-1 flex-col justify-center items-center w-full gap-8">
        <div className="flex-1 min-w-0 text-3xl text-center">
          {phaseGuideMessages[episodeStatus?.phase]}
        </div>
        <ProgressBar percent={episodeStatus?.progress} />
      </div>
      <StorageStatus
        totalCapacity={episodeStatus?.totalStorageSize * 1024 * 1024 * 1024 || 0}
        usedCapacity={episodeStatus?.usedStorageSize * 1024 * 1024 * 1024 || 0}
      />
    </div>
  );
}
