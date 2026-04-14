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

import React, { useCallback } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdDelete, MdPlayArrow, MdStop, MdDone, MdMerge } from 'react-icons/md';

import TaskPhase from '../constants/taskPhases';
import PRIMITIVE_DESCRIPTIONS from '../constants/primitiveDescriptions';
import { setPendingPrimitive } from '../features/tasks/taskSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const SegmentPanel = () => {
  const dispatch = useDispatch();
  const status = useSelector((state) => state.tasks.taskStatus);
  const pendingPrimitive = useSelector((state) => state.tasks.pendingPrimitive);
  const { sendRecordCommand } = useRosServiceCaller();

  const phase = status.phase;
  const isRecording = phase === TaskPhase.RECORDING;
  const isMerging = phase === TaskPhase.CONVERTING;
  const segmentPrimitives = status.segmentPrimitives || [];
  const segmentCount = status.segmentCount || 0;
  const hasSegments = segmentCount > 0;
  const mergeStatus = status.mergeStatus || 'none';
  const currentEpisode = status.currentEpisodeNumber || 0;

  const canAddSegment = !isRecording && !isMerging && !!pendingPrimitive;
  const canStopSegment = isRecording;
  const canFinishEpisode = !isRecording && !isMerging && hasSegments;
  const canMerge =
    !isRecording && !isMerging && hasSegments && mergeStatus !== 'pending';
  const canDiscard = !isRecording && !isMerging;

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

  const handleAdd = useCallback(() => {
    runCommand('Add segment', 'start_segment', {
      primitiveDescription: pendingPrimitive,
    });
  }, [runCommand, pendingPrimitive]);

  const handleStop = useCallback(() => {
    runCommand('Stop segment', 'stop_segment');
  }, [runCommand]);

  const handleDiscard = useCallback(
    (idx) => {
      if (!window.confirm(`Discard segment ${idx}?`)) return;
      runCommand(`Discard segment ${idx}`, 'discard_segment', {
        segmentIndex: idx,
      });
    },
    [runCommand]
  );

  const handleFinish = useCallback(() => {
    runCommand('Finish episode', 'finish_episode');
  }, [runCommand]);

  const handleMerge = useCallback(() => {
    runCommand('Merge episode', 'merge_episode', {
      episodeIndex: currentEpisode,
    });
  }, [runCommand, currentEpisode]);

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

  const classBtn = (enabled, color = 'blue') =>
    clsx(
      'px-2.5',
      'py-1',
      'rounded-md',
      'text-sm',
      'font-semibold',
      'transition-colors',
      {
        [`bg-${color}-500 text-white hover:bg-${color}-600`]: enabled,
        'bg-gray-200 text-gray-400 cursor-not-allowed': !enabled,
      }
    );

  return (
    <div className={classPanel}>
      <div className="flex items-center justify-between mb-3">
        <div className="text-lg font-semibold text-gray-800">
          Segments
        </div>
        <div className="text-xs text-gray-500">
          Episode <span className="font-bold text-gray-700">{currentEpisode}</span>
        </div>
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
              <span className="text-sm text-gray-800 truncate">{prim || '—'}</span>
            </div>
            <button
              onClick={() => handleDiscard(i)}
              disabled={!canDiscard}
              className={clsx(
                'p-1 rounded hover:bg-red-50 text-red-500',
                { 'opacity-30 cursor-not-allowed hover:bg-transparent': !canDiscard }
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

        {!isRecording && !hasSegments && (
          <div className="text-xs text-gray-400 italic px-2 py-1">
            No segments yet. Select a primitive and click Add Segment.
          </div>
        )}
      </div>

      {/* Primitive picker */}
      <div className="flex items-center gap-2 mb-2">
        <span className="text-sm text-gray-600 shrink-0">Next primitive</span>
        <select
          className={clsx(
            'flex-1 text-sm p-1.5 border border-gray-300 rounded-md',
            'focus:outline-none focus:ring-2 focus:ring-blue-500',
            { 'bg-gray-100 cursor-not-allowed': isRecording || isMerging }
          )}
          value={pendingPrimitive}
          onChange={(e) => dispatch(setPendingPrimitive(e.target.value))}
          disabled={isRecording || isMerging}
        >
          <option value="">-- Select --</option>
          {PRIMITIVE_DESCRIPTIONS.map((p) => (
            <option key={p} value={p}>
              {p}
            </option>
          ))}
        </select>
      </div>

      {/* Action buttons */}
      <div className="grid grid-cols-2 gap-2 mt-2">
        {!isRecording ? (
          <button
            onClick={handleAdd}
            disabled={!canAddSegment}
            className={classBtn(canAddSegment, 'blue')}
          >
            <div className="flex items-center justify-center gap-1">
              <MdPlayArrow size={16} />
              Add Segment
            </div>
          </button>
        ) : (
          <button
            onClick={handleStop}
            disabled={!canStopSegment}
            className={classBtn(canStopSegment, 'red')}
          >
            <div className="flex items-center justify-center gap-1">
              <MdStop size={16} />
              Stop Segment
            </div>
          </button>
        )}

        <button
          onClick={handleFinish}
          disabled={!canFinishEpisode}
          className={classBtn(canFinishEpisode, 'green')}
        >
          <div className="flex items-center justify-center gap-1">
            <MdDone size={16} />
            Finish Episode
          </div>
        </button>

        <button
          onClick={handleMerge}
          disabled={!canMerge}
          className={clsx(classBtn(canMerge, 'indigo'), 'col-span-2')}
        >
          <div className="flex items-center justify-center gap-1">
            <MdMerge size={16} />
            Merge to MCAP
            {mergeStatus && mergeStatus !== 'none' && (
              <span className="ml-1 text-xs opacity-80">({mergeStatus})</span>
            )}
          </div>
        </button>
      </div>
    </div>
  );
};

export default SegmentPanel;
