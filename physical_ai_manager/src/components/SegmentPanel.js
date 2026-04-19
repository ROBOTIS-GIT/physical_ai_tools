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

import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import {
  MdFiberManualRecord,
  MdSave,
  MdClose,
  MdDone,
  MdDelete,
  MdDeleteSweep,
} from 'react-icons/md';

import TaskPhase from '../constants/taskPhases';
import PRIMITIVE_DESCRIPTIONS from '../constants/primitiveDescriptions';
import { setPendingPrimitive } from '../features/tasks/taskSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import InfoPanel from './InfoPanel';
import Tooltip from './Tooltip';

const isInputFocused = () => {
  const el = document.activeElement;
  if (!el) return false;
  const tag = el.tagName.toLowerCase();
  return (
    tag === 'input' ||
    tag === 'textarea' ||
    tag === 'select' ||
    el.contentEditable === 'true'
  );
};

const SegmentPanel = () => {
  const dispatch = useDispatch();
  const status = useSelector((state) => state.tasks.taskStatus);
  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const pendingPrimitive = useSelector((state) => state.tasks.pendingPrimitive);
  const { sendRecordCommand } = useRosServiceCaller();

  const [hovered, setHovered] = useState(null);
  const [pressed, setPressed] = useState(null);

  // Optimistic "recording" flag: flips immediately on Record click so the
  // button set reflects user intent without waiting for the TaskStatus
  // round-trip. Reconciled with the server's phase once it echoes back.
  const [optimisticRecording, setOptimisticRecording] = useState(false);
  const optimisticRef = useRef(false);
  const [savingInProgress, setSavingInProgress] = useState(false);

  const phase = status.phase;
  const serverRecording = phase === TaskPhase.RECORDING;
  const isRecording = serverRecording || optimisticRecording;
  const segmentPrimitives = status.segmentPrimitives || [];
  const segmentCount = status.segmentCount || 0;
  const hasSegments = segmentCount > 0;

  // Keep local flag in sync whenever the server phase settles.
  useEffect(() => {
    optimisticRef.current = serverRecording;
    setOptimisticRecording(serverRecording);
  }, [serverRecording]);

  const taskInfoComplete = Boolean(
    (taskInfo.taskNum || '').trim() &&
      (taskInfo.taskName || '').trim() &&
      (taskInfo.taskInstruction?.[0] || '').trim()
  );

  const canRecord =
    !isRecording &&
    !savingInProgress &&
    !!pendingPrimitive &&
    taskInfoComplete;
  const canSave = isRecording;
  const canDiscard = isRecording || hasSegments;
  const canFinishEpisode = !isRecording && !savingInProgress && hasSegments;
  const canDiscardEpisode = !isRecording && hasSegments;

  const runCommand = useCallback(
    async (label, cmd, opts = {}) => {
      try {
        const result = await sendRecordCommand(cmd, opts);
        if (result && result.success) {
          toast.success(`${label}: ${result.message || 'OK'}`);
        } else {
          toast.error(`${label} failed: ${result?.message || 'Unknown error'}`);
        }
        return result;
      } catch (e) {
        toast.error(`${label} failed: ${e.message || e}`);
        return null;
      }
    },
    [sendRecordCommand]
  );

  const handleRecord = useCallback(async () => {
    if (!canRecord) return;
    if (!pendingPrimitive) {
      toast.error('Select a primitive first');
      return;
    }
    // Optimistic flip — buttons update before the status round-trip.
    optimisticRef.current = true;
    setOptimisticRecording(true);
    const result = await runCommand('Record', 'start_segment', {
      primitiveDescription: pendingPrimitive,
    });
    if (!result || result.success === false) {
      optimisticRef.current = false;
      setOptimisticRecording(false);
    }
  }, [canRecord, runCommand, pendingPrimitive]);

  const handleSave = useCallback(async () => {
    if (!canSave) return;
    setSavingInProgress(true);
    optimisticRef.current = false;
    setOptimisticRecording(false);
    const result = await runCommand('Save', 'stop_segment');
    setSavingInProgress(false);
    if (!result || result.success === false) {
      toast.error('Save may have failed — check server logs');
    }
  }, [canSave, runCommand]);

  const handleDiscardAction = useCallback(async () => {
    if (!canDiscard) return;
    if (isRecording) {
      // Cancel current segment: stop then remove just-finalized segment.
      const cur = status.currentSegmentIndex || 0;
      const stop = await sendRecordCommand('stop_segment');
      if (!stop || stop.success === false) {
        toast.error(`Discard failed at stop: ${stop?.message || ''}`);
        return;
      }
      optimisticRef.current = false;
      setOptimisticRecording(false);
      await runCommand('Discard', 'discard_segment', { segmentIndex: cur });
    } else {
      await runCommand('Discard', 'discard_segment', {
        segmentIndex: segmentCount - 1,
      });
    }
  }, [
    canDiscard,
    isRecording,
    segmentCount,
    sendRecordCommand,
    runCommand,
    status.currentSegmentIndex,
  ]);

  const handleDiscardSegment = useCallback(
    (idx) => {
      if (!window.confirm(`Discard segment ${idx}?`)) return;
      runCommand(`Discard #${idx}`, 'discard_segment', { segmentIndex: idx });
    },
    [runCommand]
  );

  const handleDiscardEpisode = useCallback(() => {
    if (!canDiscardEpisode) return;
    if (!window.confirm('Discard ALL pending segments?')) return;
    runCommand('Discard episode', 'discard_episode');
  }, [canDiscardEpisode, runCommand]);

  const handleFinish = useCallback(() => {
    if (!canFinishEpisode) return;
    runCommand('Finish episode', 'finish_episode');
  }, [canFinishEpisode, runCommand]);

  // Keyboard shortcuts — matching the legacy RecordControlPanel bindings.
  const handleKeyAction = useCallback(
    (e) => {
      if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (canRecord) return 'Record';
      }
      if (
        (e.ctrlKey || e.metaKey) &&
        e.shiftKey &&
        (e.key === 'x' || e.key === 'X')
      ) {
        if (canSave) return 'Save';
      }
      if (e.key === 'Escape') {
        if (canDiscard) return 'Discard';
      }
      return null;
    },
    [canRecord, canSave, canDiscard]
  );

  useEffect(() => {
    const onKeyDown = (e) => {
      if (e.repeat || isInputFocused()) return;
      const action = handleKeyAction(e);
      if (action) setPressed(action);
    };
    const onKeyUp = (e) => {
      setPressed(null);
      if (isInputFocused()) return;
      const action = handleKeyAction(e);
      if (action === 'Record') handleRecord();
      else if (action === 'Save') handleSave();
      else if (action === 'Discard') handleDiscardAction();
    };
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
    };
  }, [handleKeyAction, handleRecord, handleSave, handleDiscardAction]);

  const classPanel = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-md',
    'p-4',
    'w-full',
    'max-w-[350px]',
    'mt-3'
  );

  const classRow = clsx(
    'flex',
    'items-center',
    'justify-between',
    'gap-2',
    'px-2',
    'py-1.5',
    'rounded-md',
    'border',
    'border-gray-100'
  );

  const classMainBtn = (label, isDisabled) =>
    clsx(
      'rounded-lg',
      'border-none',
      'cursor-pointer',
      'px-2',
      'py-1.5',
      'flex',
      'items-center',
      'justify-center',
      'gap-1',
      'bg-gray-100',
      'transition-all',
      'duration-150',
      'font-semibold',
      'text-sm',
      'flex-1',
      {
        'bg-gray-400': pressed === label && !isDisabled,
        'bg-gray-200': hovered === label && pressed !== label && !isDisabled,
        'opacity-30 cursor-not-allowed bg-gray-50': isDisabled,
      }
    );

  // NOTE: Tailwind JIT only picks up class names it can see literally in
  // source, so the enabled-color classes must be written out.
  const SECONDARY_COLOR_CLASSES = {
    indigo: 'bg-indigo-500 text-white hover:bg-indigo-600',
    red: 'bg-red-500 text-white hover:bg-red-600',
    green: 'bg-green-500 text-white hover:bg-green-600',
  };
  const secondaryBtn = (enabled, color) =>
    clsx(
      'px-2.5',
      'py-1.5',
      'rounded-md',
      'text-sm',
      'font-semibold',
      'transition-colors',
      'flex',
      'items-center',
      'justify-center',
      'gap-1',
      enabled
        ? SECONDARY_COLOR_CLASSES[color] || SECONDARY_COLOR_CLASSES.indigo
        : 'bg-gray-200 text-gray-400 cursor-not-allowed'
    );

  const mainButtons = [
    {
      label: 'Record',
      icon: MdFiberManualRecord,
      color: '#d32f2f',
      enabled: canRecord,
      handler: handleRecord,
      description: 'Start a new segment',
      shortcut: 'Space',
    },
    {
      label: 'Save',
      icon: MdSave,
      color: '#388e3c',
      enabled: canSave,
      handler: handleSave,
      description: 'Stop current segment and add to list',
      shortcut: 'Ctrl+Shift+X',
    },
    {
      label: 'Discard',
      icon: MdClose,
      color: '#757575',
      enabled: canDiscard,
      handler: handleDiscardAction,
      description: isRecording
        ? 'Cancel current segment'
        : 'Discard last segment',
      shortcut: 'Escape',
    },
  ];

  return (
    <div className={classPanel}>
      <div className="mb-3 text-lg font-semibold text-gray-800">Rosbag Recorder</div>

      {/* Task Information block (fill first, then record) */}
      <div className="mb-3">
        <div className="text-sm font-semibold text-gray-700 mb-2">
          Task Information
          <span className="ml-1 text-xs font-normal text-gray-400">
            (required before recording)
          </span>
        </div>
        <InfoPanel variant="embedded" />
      </div>

      {/* Primitive picker */}
      <div className="flex items-center gap-2 mb-3">
        <span className="text-sm text-gray-600 shrink-0">Next primitive</span>
        <select
          className={clsx(
            'flex-1 text-sm p-1.5 border border-gray-300 rounded-md',
            'focus:outline-none focus:ring-2 focus:ring-blue-500',
            { 'bg-gray-100 cursor-not-allowed': isRecording }
          )}
          value={pendingPrimitive}
          onChange={(e) => dispatch(setPendingPrimitive(e.target.value))}
          disabled={isRecording}
        >
          <option value="">-- Select --</option>
          {PRIMITIVE_DESCRIPTIONS.map((p) => (
            <option key={p} value={p}>
              {p}
            </option>
          ))}
        </select>
      </div>

      {/* Segment list */}
      <div className="flex flex-col gap-1.5 mb-3">
        {segmentPrimitives.map((prim, i) => (
          <div key={`seg-${i}`} className={classRow}>
            <div className="flex items-center gap-2 min-w-0">
              <MdDone className="text-green-500 shrink-0" />
              <span className="text-sm font-mono text-gray-500 shrink-0">
                #{i}
              </span>
              <span className="text-sm text-gray-800 truncate">
                {prim || '—'}
              </span>
            </div>
            <button
              onClick={() => handleDiscardSegment(i)}
              disabled={isRecording}
              className={clsx(
                'p-1 rounded hover:bg-red-50 text-red-500',
                {
                  'opacity-30 cursor-not-allowed hover:bg-transparent':
                    isRecording,
                }
              )}
              aria-label={`Discard segment ${i}`}
              title={`Discard segment ${i}`}
            >
              <MdDelete size={18} />
            </button>
          </div>
        ))}

        {isRecording && (
          <div className={clsx(classRow, 'bg-red-50 border-red-200')}>
            <div className="flex items-center gap-2 min-w-0">
              <span className="h-2 w-2 rounded-full bg-red-500 animate-pulse shrink-0" />
              <span className="text-sm font-mono text-gray-500 shrink-0">
                #{status.currentSegmentIndex}
              </span>
              <span className="text-sm text-gray-800 truncate">
                {pendingPrimitive || '—'} (recording {status.proceedTime}s)
              </span>
            </div>
          </div>
        )}

        {savingInProgress && (
          <div className={clsx(classRow, 'bg-amber-50 border-amber-200')}>
            <div className="flex items-center gap-2 min-w-0">
              <span className="h-2 w-2 rounded-full bg-amber-500 animate-pulse shrink-0" />
              <span className="text-sm text-amber-800 font-medium">
                Saving segment...
              </span>
            </div>
          </div>
        )}

        {!isRecording && !hasSegments && (
          <div className="text-xs text-gray-400 italic px-2 py-1">
            No segments yet. Select a primitive and press Record.
          </div>
        )}
      </div>

      {/* Main Record / Save / Discard buttons (3-button set) */}
      <div className="flex items-center gap-1.5 mb-3">
        {mainButtons.map(
          ({ label, icon: Icon, color, enabled, handler, description, shortcut }) => {
            const isDisabled = !enabled;
            return (
              <Tooltip
                key={label}
                position="top"
                content={
                  <div className="text-center">
                    <div className="font-semibold">{description}</div>
                    {!isDisabled && (
                      <div className="text-sm mt-1 text-gray-300">
                        <span className="font-mono bg-gray-700 px-1 rounded">
                          {shortcut}
                        </span>
                      </div>
                    )}
                  </div>
                }
                disabled={false}
                className="relative flex-1"
              >
                <button
                  className={classMainBtn(label, isDisabled)}
                  onClick={() => !isDisabled && handler()}
                  onMouseEnter={() => !isDisabled && setHovered(label)}
                  onMouseLeave={() => {
                    setHovered(null);
                    setPressed(null);
                  }}
                  onMouseDown={() => !isDisabled && setPressed(label)}
                  onMouseUp={() => setPressed(null)}
                  disabled={isDisabled}
                  aria-label={description}
                >
                  <Icon
                    style={{ fontSize: '1.1rem' }}
                    color={isDisabled ? '#9ca3af' : color}
                  />
                  {label}
                </button>
              </Tooltip>
            );
          }
        )}
      </div>

      {/* Finish Episode / Discard Episode */}
      <div className="flex flex-col gap-2">
        <button
          onClick={handleFinish}
          disabled={!canFinishEpisode}
          className={clsx(secondaryBtn(canFinishEpisode, 'indigo'), 'w-full')}
        >
          <MdDone size={16} />
          Finish Episode
        </button>

        <button
          onClick={handleDiscardEpisode}
          disabled={!canDiscardEpisode}
          className={secondaryBtn(canDiscardEpisode, 'red')}
        >
          <MdDeleteSweep size={16} />
          Discard Episode
        </button>
      </div>
    </div>
  );
};

export default SegmentPanel;
