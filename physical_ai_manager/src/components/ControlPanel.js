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
import CompactSystemStatus from './CompactSystemStatus';
import SystemStatus from './SystemStatus';
import EpisodeStatus from './EpisodeStatus';
import Tooltip from './Tooltip';

const buttons = [
  {
    label: 'Start',
    icon: MdPlayArrow,
    color: '#1976d2',
    description: 'Start recording task',
    shortcut: 'Space',
  },
  {
    label: 'Stop',
    icon: MdStop,
    color: '#d32f2f',
    description: 'Stop current task',
    shortcut: 'Space',
  },
  {
    label: 'Retry',
    icon: MdReplay,
    color: '#fbc02d',
    description: 'Retry current episode',
    shortcut: '←',
  },
  {
    label: 'Next',
    icon: MdSkipNext,
    color: '#388e3c',
    description: 'Move to next episode',
    shortcut: '→',
  },
  {
    label: 'Finish',
    icon: MdCheck,
    color: '#388e3c',
    description: 'Finish and save task',
    shortcut: 'Ctrl+C',
  },
];

const phaseGuideMessages = {
  0: '📍 Ready to start',
  1: '🔥 Warmup in progress',
  2: '🏠 Reset in progress',
  3: '🔴 Recording in progress',
  4: '💾 Saving...',
  5: '◼️ Task Stopped',
  6: '⚡ Inference in progress',
};

const spinnerFrames = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧'];

export default function ControlPanel({ onCommand, episodeStatus, taskInfo }) {
  const [hovered, setHovered] = useState(null);
  const [pressed, setPressed] = useState(null);
  const [started, setStarted] = useState(false);
  const [expandedSystemIndex, setExpandedSystemIndex] = useState(null);
  const [spinnerIndex, setSpinnerIndex] = useState(0);
  const startedRef = useRef(started);

  useEffect(() => {
    startedRef.current = started;
  }, [started]);

  // Spinner animation effect - update whenever taskStatus changes (ROS topic received)
  useEffect(() => {
    // Update spinner index whenever episodeStatus changes (regardless of phase)
    updateSpinnerFrame();
  }, [episodeStatus]);

  const isReadyState = (phase) => {
    return phase === 0;
  };

  const isStoppedState = (phase) => {
    return phase === 5;
  };

  const isRunningState = (phase) => {
    return phase === 1 || phase === 2 || phase === 3 || phase === 4 || phase === 6;
  };

  const updateSpinnerFrame = () => {
    setSpinnerIndex((prevIndex) => (prevIndex + 1) % spinnerFrames.length);
  };

  // Check if button should be enabled based on phase
  const isButtonEnabled = useCallback(
    (label) => {
      const phase = episodeStatus?.phase;

      switch (label) {
        case 'Start':
          // Start button disabled when task is running or when running flag is true
          return (isReadyState(phase) || isStoppedState(phase)) && !episodeStatus?.running;
        case 'Stop':
          // Stop button enabled only when task is running
          return !isReadyState(phase) && !isStoppedState(phase);
        case 'Retry':
          // Retry button enabled only when task is stopped
          return !isReadyState(phase);
        case 'Next':
          // Next button enabled only when task is stopped
          return !isReadyState(phase);
        case 'Finish':
          // Finish button enabled only when task is stopped
          return true; // Always enabled
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
      if (label === 'Stop' || label === 'Finish') setStarted(false);
    },
    [onCommand]
  );

  // Helper function to get button label from keyboard event
  const getButtonFromKey = useCallback(
    (e) => {
      if (e.key === 'ArrowLeft' && isButtonEnabled('Retry')) {
        return 'Retry';
      } else if (e.key === 'ArrowRight' && isButtonEnabled('Next')) {
        return 'Next';
      } else if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (isButtonEnabled('Start')) {
          return 'Start';
        } else if (isButtonEnabled('Stop')) {
          return 'Stop';
        }
      } else if (
        (e.ctrlKey || e.metaKey) &&
        (e.key === 'c' || e.key === 'C') &&
        isButtonEnabled('Finish')
      ) {
        return 'Finish';
      }
      return null;
    },
    [isButtonEnabled]
  );

  // Add keyboard press visual feedback
  const handleKeyboardPress = useCallback((buttonLabel) => {
    if (buttonLabel) {
      setPressed(buttonLabel);
    }
  }, []);

  // Add keyboard release visual feedback
  const handleKeyboardRelease = useCallback(() => {
    setPressed(null);
  }, []);

  useEffect(() => {
    const isInputFocused = () => {
      const activeElement = document.activeElement;
      if (!activeElement) return false;

      const tagName = activeElement.tagName.toLowerCase();
      const isEditable = activeElement.contentEditable === 'true';

      return tagName === 'input' || tagName === 'textarea' || tagName === 'select' || isEditable;
    };

    const handleKeyDown = (e) => {
      // Ignore repeated keydown events when holding the key
      if (e.repeat) return;

      // Ignore keyboard shortcuts when user is typing in input fields
      if (isInputFocused()) return;

      const buttonLabel = getButtonFromKey(e);
      if (buttonLabel) {
        handleKeyboardPress(buttonLabel);
      }
    };

    const handleKeyUp = (e) => {
      // Always release pressed state on keyup
      handleKeyboardRelease();

      // Ignore keyboard shortcuts when user is typing in input fields
      if (isInputFocused()) return;

      // Get the button label and execute the command
      const buttonLabel = getButtonFromKey(e);
      if (buttonLabel) {
        handleCommand(buttonLabel);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [
    handleCommand,
    isButtonEnabled,
    getButtonFromKey,
    handleKeyboardPress,
    handleKeyboardRelease,
  ]);

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
      'w-full',
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

  const handleButtonKeyUp = (e, label, isDisabled) => {
    if (isDisabled) return;
    if (e.key === 'Enter') {
      handleCommand(label);
    }
  };

  const handleButtonKeyDown = (e, label, isDisabled) => {
    if (isDisabled) return;
    if (e.key === ' ') {
      e.preventDefault(); // Prevent scrolling
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
        {buttons.map(({ label, icon: Icon, color, description, shortcut }) => {
          const isDisabled = !isButtonEnabled(label);

          const tooltipContent = (
            <div className="text-center">
              <div className="font-semibold text-lg">{description}</div>
              {!isDisabled && (
                <div className="text-md mt-1 text-gray-300">
                  Press <span className="font-mono bg-gray-700 px-1 rounded">{shortcut}</span>
                </div>
              )}
              {isDisabled && <div className="text-xs mt-1 text-red-300">Currently disabled</div>}
            </div>
          );

          return (
            <Tooltip
              key={label}
              content={tooltipContent}
              disabled={false}
              className="whitespace-normal max-w-48"
            >
              <button
                className={classControlPanelButtons(label, isDisabled)}
                style={{ fontFamily: 'Pretendard Variable', fontSize: 'clamp(1rem, 2vw, 2.2rem)' }}
                tabIndex={isDisabled ? -1 : 0}
                onClick={() => !isDisabled && handleCommand(label)}
                onKeyUp={(e) => handleButtonKeyUp(e, label, isDisabled)}
                onKeyDown={(e) => handleButtonKeyDown(e, label, isDisabled)}
                onMouseEnter={() => handleMouseEnter(label, isDisabled)}
                onMouseLeave={handleMouseLeave}
                onMouseDown={() => handleMouseDown(label, isDisabled)}
                onMouseUp={handleMouseUp}
                disabled={isDisabled}
              >
                <span className={classControlPanelButtonIcon}>
                  <Icon
                    style={{ fontSize: 'clamp(1rem, 4vw, 4rem)' }}
                    color={isDisabled ? '#9ca3af' : color}
                  />
                </span>
                {label}
              </button>
            </Tooltip>
          );
        })}
      </div>
      <div className="w-full h-full rounded-2xl flex flex-1 flex-col justify-center items-center gap-2">
        <div className="flex items-center justify-center gap-5">
          <div
            className="flex min-w-0 text-center items-center gap-2"
            style={{ fontSize: 'clamp(1rem, 2vw, 2.5rem)' }}
          >
            {phaseGuideMessages[episodeStatus?.phase]}
          </div>
          <div>
            {/* Spinner */}
            {isRunningState(episodeStatus?.phase) && (
              <span className="font-mono text-blue-500 text-4xl">
                {spinnerFrames[spinnerIndex]}
              </span>
            )}
          </div>
        </div>
        <div className="w-full flex flex-col items-center gap-1">
          <div className="w-full max-w-xl flex flex-col items-center gap-1">
            <div className="flex px-3 w-full justify-end text-xl text-gray-500 font-bold whitespace-nowrap ">
              {episodeStatus?.proceedTime} / {episodeStatus?.totalTime} (s)
            </div>
            <ProgressBar percent={episodeStatus?.progress} />
          </div>
        </div>
      </div>
      <div className="flex justify-end flex-[0.4] min-w-30 h-full p-1">
        <EpisodeStatus
          episodeStatus={{
            ...episodeStatus,
            numEpisodes: taskInfo?.numEpisodes,
          }}
        />
      </div>
      <div className="flex flex-col gap-2">
        {expandedSystemIndex !== null ? (
          /* Expanded System View */
          <div onClick={() => setExpandedSystemIndex(null)} className="cursor-pointer">
            {expandedSystemIndex === 0 ? (
              /* CPU Details */
              <SystemStatus
                label="CPU"
                type="cpu"
                cpuPercentage={episodeStatus?.usedCpu} // CPU usage percentage
              />
            ) : expandedSystemIndex === 1 ? (
              /* RAM Details */
              <SystemStatus
                label="RAM"
                type="ram"
                totalCapacity={episodeStatus?.totalRamSize * 1024 * 1024 * 1024 || 0}
                usedCapacity={episodeStatus?.usedRamSize * 1024 * 1024 * 1024 || 0}
              />
            ) : (
              /* Storage Details */
              <SystemStatus
                label="Storage"
                type="storage"
                totalCapacity={episodeStatus?.totalStorageSize * 1024 * 1024 * 1024 || 0}
                usedCapacity={episodeStatus?.usedStorageSize * 1024 * 1024 * 1024 || 0}
              />
            )}
          </div>
        ) : (
          /* Compact System List */
          <>
            {/* CPU */}
            <div onClick={() => setExpandedSystemIndex(0)} className="cursor-pointer">
              <CompactSystemStatus
                label="CPU"
                type="cpu"
                cpuPercentage={episodeStatus?.usedCpu} // CPU usage percentage
              />
            </div>

            {/* RAM */}
            <div onClick={() => setExpandedSystemIndex(1)} className="cursor-pointer">
              <CompactSystemStatus
                label="RAM"
                type="ram"
                totalCapacity={episodeStatus?.totalRamSize * 1024 * 1024 * 1024 || 0}
                usedCapacity={episodeStatus?.usedRamSize * 1024 * 1024 * 1024 || 0}
              />
            </div>

            {/* Storage */}
            <div onClick={() => setExpandedSystemIndex(2)} className="cursor-pointer">
              <CompactSystemStatus
                label="Storage"
                type="storage"
                totalCapacity={episodeStatus?.totalStorageSize * 1024 * 1024 * 1024 || 0}
                usedCapacity={episodeStatus?.usedStorageSize * 1024 * 1024 * 1024 || 0}
              />
            </div>
          </>
        )}
      </div>
    </div>
  );
}
