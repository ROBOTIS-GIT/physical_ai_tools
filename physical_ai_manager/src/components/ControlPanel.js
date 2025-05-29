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

const buttons = [
  { label: 'Start', icon: MdPlayArrow, color: '#1976d2' },
  { label: 'Stop', icon: MdStop, color: '#d32f2f' },
  { label: 'Retry', icon: MdReplay, color: '#fbc02d' },
  { label: 'Next', icon: MdSkipNext, color: '#388e3c' },
  { label: 'Finish', icon: MdCheck, color: '#388e3c' },
];

export default function ControlPanel({ onCommand }) {
  const icon_size = 70;
  const [hovered, setHovered] = useState(null);
  const [pressed, setPressed] = useState(null);
  const [started, setStarted] = useState(false);
  const startedRef = useRef(started);

  useEffect(() => {
    startedRef.current = started;
  }, [started]);

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
      if (e.key === 'ArrowLeft') {
        handleCommand('Retry');
      } else if (e.key === 'ArrowRight') {
        handleCommand('Next');
      } else if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (!startedRef.current) {
          handleCommand('Start');
        } else {
          handleCommand('Stop');
        }
      } else if ((e.ctrlKey || e.metaKey) && (e.key === 'c' || e.key === 'C')) {
        handleCommand('Finish');
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleCommand]);

  return (
    <div
      className={clsx(
        'h-56',
        'bg-gray-300',
        'rounded-3xl',
        'mx-8',
        'my-4',
        'p-4',
        'flex',
        'flex-row',
        'items-center',
        'gap-2',
        'shadow-lg'
      )}
    >
      <div className="flex flex-[2] items-center w-full h-full gap-4">
        {buttons.map(({ label, icon: Icon, color }) => {
          return (
            <button
              key={label}
              className={clsx(
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
                {
                  'bg-gray-300': pressed === label,
                  'bg-gray-200': hovered === label && pressed !== label,
                  'bg-gray-100': hovered !== label && pressed !== label,
                }
              )}
              style={{ fontFamily: 'Pretendard Variable' }}
              tabIndex={0}
              onClick={() => handleCommand(label)}
              onKeyDown={(e) => {
                if (e.key === 'Enter') {
                  handleCommand(label);
                }
                if (e.key === ' ') {
                  e.preventDefault();
                }
              }}
              onMouseEnter={() => setHovered(label)}
              onMouseLeave={() => {
                setHovered(null);
                setPressed(null);
              }}
              onMouseDown={() => setPressed(label)}
              onMouseUp={() => setPressed(null)}
            >
              <span
                className={clsx(
                  'bg-transparent',
                  'rounded-full',
                  'w-20',
                  'h-20',
                  'flex',
                  'items-center',
                  'justify-center',
                  'mb-1'
                )}
              >
                <Icon size={icon_size} color={color} />
              </span>
              {label}
            </button>
          );
        })}
      </div>
      <div className="flex flex-1 flex-col justify-center items-center w-full gap-8">
        <div className="flex-1 min-w-0 text-3xl text-center">Please prepare next episode</div>
        <ProgressBar percent={60} />
      </div>
    </div>
  );
}
