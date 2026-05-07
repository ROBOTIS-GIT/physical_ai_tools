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

import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import {
  MdArrowDropDown,
  MdArrowDropUp,
  MdFiberManualRecord,
  MdSave,
  MdDone,
  MdDelete,
  MdDeleteSweep,
} from 'react-icons/md';

import TaskPhase from '../constants/taskPhases';
import {
  setPendingSubTask,
  setPlannedCount,
  setPlannedSubTasks,
  setPlannedSubTaskAt,
  setSlotToServerIdx,
  setActiveSlotIndex,
  resetSegmentPlan,
} from '../features/tasks/taskSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import InfoPanel from './InfoPanel';
import Tooltip from './Tooltip';

const MAX_PLANNED_SLOTS = 50;

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
  const { sendRecordCommand } = useRosServiceCaller();

  // Optimistic "recording" flag: flips immediately on Record click so the
  // button set reflects user intent without waiting for the TaskStatus
  // round-trip. Reconciled with the server's phase once it echoes back.
  const [optimisticRecording, setOptimisticRecording] = useState(false);
  const [savingInProgress, setSavingInProgress] = useState(false);

  // Plan-mode state — pre-decided slot count and sub_tasks for an episode.
  // Lives in Redux so it survives navigating away from the recorder panel
  // (e.g. Home → Record), matching how taskInfo persists. plannedCount === 0
  // means no plan yet. -1 in slotToServerIdx means the slot has not been
  // recorded yet; otherwise the value is the backend segment idx.
  const plannedCount = useSelector((state) => state.tasks.plannedCount);
  const plannedSubTasks = useSelector((state) => state.tasks.plannedSubTasks);
  const slotToServerIdx = useSelector((state) => state.tasks.slotToServerIdx);
  const activeSlotIndex = useSelector((state) => state.tasks.activeSlotIndex);

  const phase = status.phase;
  const serverRecording = phase === TaskPhase.RECORDING;
  const isRecording = serverRecording || optimisticRecording;
  const segmentCount = status.segmentCount || 0;
  const hasSegments = segmentCount > 0;

  // Keep local flag in sync whenever the server phase settles.
  useEffect(() => {
    setOptimisticRecording(serverRecording);
  }, [serverRecording]);

  const taskInfoComplete = Boolean(
    (taskInfo.taskNum || '').trim() &&
      (taskInfo.taskName || '').trim() &&
      (taskInfo.taskInstruction?.[0] || '').trim()
  );

  const isPlanMode = plannedCount > 0;
  const firstPendingSlot = useMemo(
    () => slotToServerIdx.findIndex((v) => v === -1),
    [slotToServerIdx]
  );
  const planComplete = isPlanMode && firstPendingSlot === -1;
  const numSavedInPlan = useMemo(
    () => slotToServerIdx.filter((v) => v >= 0).length,
    [slotToServerIdx]
  );
  const allSubTasksFilled = useMemo(
    () =>
      isPlanMode &&
      plannedSubTasks.length === plannedCount &&
      plannedSubTasks.every((s) => !!(s && s.trim())),
    [isPlanMode, plannedSubTasks, plannedCount]
  );

  const canStartRecord =
    isPlanMode &&
    !isRecording &&
    !savingInProgress &&
    allSubTasksFilled &&
    !planComplete &&
    activeSlotIndex >= 0 &&
    activeSlotIndex < plannedCount &&
    taskInfoComplete;

  const canFinishEpisode = !isRecording && !savingInProgress && hasSegments;
  const canDiscardEpisode = !isRecording && !savingInProgress && hasSegments;
  const canResetPlan =
    isPlanMode && !isRecording && !savingInProgress && numSavedInPlan === 0;

  // Sync redux pendingSubTask so InfoPanel's debounced set_task_info push
  // carries the right sub_task for the slot we are about to record.
  useEffect(() => {
    if (isPlanMode && !planComplete && activeSlotIndex < plannedCount) {
      dispatch(setPendingSubTask(plannedSubTasks[activeSlotIndex] || ''));
    } else if (!isPlanMode) {
      dispatch(setPendingSubTask(''));
    }
  }, [
    isPlanMode,
    planComplete,
    activeSlotIndex,
    plannedCount,
    plannedSubTasks,
    dispatch,
  ]);

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

  // Full plan reset — clears slot count, sub_tasks, and progress. Used by
  // the Reset button.
  const resetPlanState = useCallback(() => {
    dispatch(resetSegmentPlan());
  }, [dispatch]);

  // Episode-progress reset — keeps the planned slots and sub_tasks, just
  // marks all slots as not-yet-recorded so the same plan can be re-used for
  // the next episode. Used after Finish/Discard Episode.
  const resetEpisodeProgress = useCallback(() => {
    dispatch(setSlotToServerIdx(slotToServerIdx.map(() => -1)));
    dispatch(setActiveSlotIndex(0));
  }, [dispatch, slotToServerIdx]);

  // Highest plan-slot index that already has a saved server segment. The
  // count cannot shrink below highestSavedIdx + 1 without orphaning recorded
  // data on the backend.
  const minAllowedCount = useMemo(() => {
    let highest = -1;
    for (let i = 0; i < slotToServerIdx.length; i++) {
      if (slotToServerIdx[i] >= 0) highest = i;
    }
    return highest + 1;
  }, [slotToServerIdx]);

  // Live-resize the plan to `n` slots. Appends empty slots when growing;
  // truncates trailing pending slots when shrinking. Caller is expected to
  // have validated against minAllowedCount.
  const applyPlanCount = useCallback(
    (n) => {
      if (n === plannedCount) return;
      let nextSubTasks;
      let nextSlotMap;
      if (n > plannedCount) {
        const add = n - plannedCount;
        nextSubTasks = [...plannedSubTasks, ...Array(add).fill('')];
        nextSlotMap = [...slotToServerIdx, ...Array(add).fill(-1)];
      } else {
        nextSubTasks = plannedSubTasks.slice(0, n);
        nextSlotMap = slotToServerIdx.slice(0, n);
      }
      dispatch(setPlannedCount(n));
      dispatch(setPlannedSubTasks(nextSubTasks));
      dispatch(setSlotToServerIdx(nextSlotMap));
      const firstPending = nextSlotMap.findIndex((v) => v === -1);
      dispatch(setActiveSlotIndex(
        firstPending >= 0 ? firstPending : Math.max(0, n - 1)
      ));
    },
    [dispatch, plannedCount, plannedSubTasks, slotToServerIdx]
  );

  const handlePlanCountInput = useCallback(
    (rawValue) => {
      if (isRecording || savingInProgress) return;
      const n = parseInt(rawValue, 10);
      if (!Number.isFinite(n) || n < 0) return;
      if (n > MAX_PLANNED_SLOTS) {
        toast.error(`Max ${MAX_PLANNED_SLOTS} subtasks per episode`);
        applyPlanCount(MAX_PLANNED_SLOTS);
        return;
      }
      if (n < minAllowedCount) {
        toast.error(
          `Cannot reduce below ${minAllowedCount} — segments are already saved`
        );
        applyPlanCount(minAllowedCount);
        return;
      }
      applyPlanCount(n);
    },
    [isRecording, savingInProgress, minAllowedCount, applyPlanCount]
  );

  const stepPlanCount = useCallback(
    (delta) => {
      handlePlanCountInput(plannedCount + delta);
    },
    [plannedCount, handlePlanCountInput]
  );

  const handleResetPlan = useCallback(() => {
    if (!canResetPlan) return;
    resetPlanState();
  }, [canResetPlan, resetPlanState]);

  const updatePlannedSubTask = useCallback(
    (idx, value) => {
      dispatch(setPlannedSubTaskAt({ index: idx, value }));
    },
    [dispatch]
  );

  const startRecordingSlot = useCallback(
    async (slotIdx) => {
      const subTask = (plannedSubTasks[slotIdx] || '').trim();
      if (!subTask) return null;
      setOptimisticRecording(true);
      const result = await runCommand('Record', 'start_segment', {
        subTask,
      });
      if (!result || result.success === false) {
        setOptimisticRecording(false);
      }
      return result;
    },
    [plannedSubTasks, runCommand]
  );

  const handleRecordStart = useCallback(async () => {
    if (!canStartRecord) return;
    await startRecordingSlot(activeSlotIndex);
  }, [canStartRecord, startRecordingSlot, activeSlotIndex]);

  const handleSlotSave = useCallback(
    async (slotIdx) => {
      if (slotIdx !== activeSlotIndex) return;
      if (!isRecording || savingInProgress) return;
      setSavingInProgress(true);
      setOptimisticRecording(false);
      const result = await runCommand('Save', 'stop_segment');
      if (!result || result.success === false) {
        setSavingInProgress(false);
        return;
      }

      // Map this plan slot to the new server segment index.
      const assignedServerIdx = slotToServerIdx.filter((v) => v >= 0).length;
      const updatedSlotMap = slotToServerIdx.map((v, i) =>
        i === slotIdx ? assignedServerIdx : v
      );
      dispatch(setSlotToServerIdx(updatedSlotMap));

      const nextPending = updatedSlotMap.findIndex((v) => v === -1);
      if (nextPending >= 0) {
        dispatch(setActiveSlotIndex(nextPending));
        await startRecordingSlot(nextPending);
      }
      // If no pending slot remains, plan is complete; user presses Finish
      // Episode (or Discard Episode) manually — we don't auto-finalize.
      setSavingInProgress(false);
    },
    [
      dispatch,
      activeSlotIndex,
      isRecording,
      savingInProgress,
      runCommand,
      slotToServerIdx,
      startRecordingSlot,
    ]
  );

  // Per-row trash: cancels the segment if this slot is currently recording,
  // otherwise discards the saved segment for this slot. No-op for pending
  // slots that have nothing to throw away.
  const handleSlotTrash = useCallback(
    async (slotIdx) => {
      if (savingInProgress) return;
      const isActiveRecording =
        slotIdx === activeSlotIndex && isRecording;
      const serverIdx = slotToServerIdx[slotIdx];

      if (isActiveRecording) {
        setOptimisticRecording(false);
        await runCommand('Discard', 'cancel_segment');
        return;
      }

      if (serverIdx < 0) return;
      if (!window.confirm(`Discard segment ${slotIdx + 1}?`)) return;
      const result = await runCommand(
        `Discard #${slotIdx + 1}`,
        'discard_segment',
        { segmentIndex: serverIdx }
      );
      if (!result || result.success === false) return;
      const updated = slotToServerIdx.map((v, i) => {
        if (i === slotIdx) return -1;
        if (v > serverIdx) return v - 1;
        return v;
      });
      dispatch(setSlotToServerIdx(updated));
      const nextPending = updated.findIndex((v) => v === -1);
      dispatch(setActiveSlotIndex(nextPending >= 0 ? nextPending : 0));
    },
    [
      dispatch,
      savingInProgress,
      activeSlotIndex,
      isRecording,
      slotToServerIdx,
      runCommand,
    ]
  );

  const handleFinish = useCallback(async () => {
    if (!canFinishEpisode) return;
    const result = await runCommand('Finish episode', 'finish_episode');
    if (result && result.success) {
      resetEpisodeProgress();
    }
  }, [canFinishEpisode, runCommand, resetEpisodeProgress]);

  const handleDiscardEpisode = useCallback(async () => {
    if (!canDiscardEpisode) return;
    if (!window.confirm('Discard ALL pending segments?')) return;
    const result = await runCommand('Discard episode', 'discard_episode');
    if (result && result.success) {
      resetEpisodeProgress();
    }
  }, [canDiscardEpisode, runCommand, resetEpisodeProgress]);

  // Keyboard shortcuts — adapted to plan-mode flow.
  // Space → Record Start, Ctrl+Shift+X → Save active slot,
  // Esc → cancel/discard via the active slot's trash.
  const handleKeyAction = useCallback(
    (e) => {
      if (e.key === ' ' || e.key === 'Spacebar' || e.code === 'Space') {
        if (canStartRecord) return 'RecordStart';
      }
      if (
        (e.ctrlKey || e.metaKey) &&
        e.shiftKey &&
        (e.key === 'x' || e.key === 'X')
      ) {
        if (isRecording && !savingInProgress) return 'Save';
      }
      if (e.key === 'Escape') {
        if (isRecording && !savingInProgress) return 'CancelActive';
      }
      return null;
    },
    [canStartRecord, isRecording, savingInProgress]
  );

  useEffect(() => {
    const onKeyUp = (e) => {
      if (isInputFocused()) return;
      const action = handleKeyAction(e);
      if (action === 'RecordStart') handleRecordStart();
      else if (action === 'Save') handleSlotSave(activeSlotIndex);
      else if (action === 'CancelActive') handleSlotTrash(activeSlotIndex);
    };
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keyup', onKeyUp);
    };
  }, [
    handleKeyAction,
    handleRecordStart,
    handleSlotSave,
    handleSlotTrash,
    activeSlotIndex,
  ]);

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

  const renderSlotRow = (i) => {
    const serverIdx = slotToServerIdx[i];
    const isSaved = serverIdx >= 0;
    const isActive = i === activeSlotIndex && !planComplete;
    const isCurrentlyRecording = isActive && isRecording;
    const dropdownDisabled = isSaved || isRecording || savingInProgress;
    const saveEnabled = isCurrentlyRecording && !savingInProgress;
    // Trash cancels the in-progress recording for the active slot, or wipes
    // out the saved segment for a saved slot. Disabled for pending slots
    // and during the brief save/transition window. A saved slot's trash is
    // only available when nothing else is recording — discard_segment must
    // not race with an active recording on the backend.
    const trashEnabled =
      !savingInProgress &&
      (isCurrentlyRecording || (isSaved && !isRecording));
    const trashTitle = isCurrentlyRecording
      ? 'Cancel current recording'
      : isSaved
      ? `Discard segment ${i + 1}`
      : 'Nothing to discard';

    return (
      <div
        key={`slot-${i}`}
        className={clsx(
          'flex items-center gap-2 px-2 py-1.5 rounded-md border',
          {
            'border-gray-100 opacity-60 bg-gray-50': isSaved,
            'border-red-300 bg-red-50': isCurrentlyRecording,
            'border-blue-200 bg-blue-50': isActive && !isCurrentlyRecording,
            'border-gray-100': !isSaved && !isActive,
          }
        )}
      >
        <span
          className={clsx(
            'text-xs font-mono w-6 shrink-0 text-center rounded flex items-center justify-center',
            {
              'bg-green-100 text-green-700': isSaved,
              'text-blue-700 font-bold': isActive && !isSaved,
              'text-gray-500': !isSaved && !isActive,
            }
          )}
        >
          {isSaved ? <MdDone size={14} /> : `#${i + 1}`}
        </span>
        <input
          type="text"
          lang="ko"
          className={clsx(
            'flex-1 text-sm p-1 border border-gray-300 rounded-md min-w-0',
            'focus:outline-none focus:ring-2 focus:ring-blue-500',
            { 'bg-gray-100 cursor-not-allowed text-gray-500': dropdownDisabled }
          )}
          value={plannedSubTasks[i] || ''}
          placeholder="sub_task 입력"
          onChange={(e) => updatePlannedSubTask(i, e.target.value)}
          disabled={dropdownDisabled}
        />
        <button
          onClick={() => handleSlotSave(i)}
          disabled={!saveEnabled}
          className={clsx(
            'px-2 py-1 rounded-md text-xs font-semibold flex items-center gap-1',
            saveEnabled
              ? 'bg-green-500 text-white hover:bg-green-600'
              : 'bg-gray-200 text-gray-400 cursor-not-allowed'
          )}
          aria-label={`Save segment ${i + 1}`}
          title="Save this segment"
        >
          <MdSave size={14} />
          Save
        </button>
        <button
          onClick={() => handleSlotTrash(i)}
          disabled={!trashEnabled}
          className={clsx(
            'p-1 rounded',
            trashEnabled
              ? 'hover:bg-red-50 text-red-500'
              : 'text-gray-300 cursor-not-allowed'
          )}
          aria-label={trashTitle}
          title={trashTitle}
        >
          <MdDelete size={16} />
        </button>
      </div>
    );
  };

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

      {/* Record Start — single big button that drives the whole episode */}
      <Tooltip
        position="top"
        content={
          <div className="text-center">
            <div className="font-semibold">
              {isPlanMode
                ? 'Start recording from the next pending slot'
                : 'Set the number of subtasks below first'}
            </div>
            {canStartRecord && (
              <div className="text-sm mt-1 text-gray-300">
                <span className="font-mono bg-gray-700 px-1 rounded">Space</span>
              </div>
            )}
          </div>
        }
        disabled={false}
        className="block w-full"
      >
        <button
          onClick={handleRecordStart}
          disabled={!canStartRecord}
          className={clsx(
            'w-full mb-3 px-3 py-2.5 rounded-lg font-semibold text-sm',
            'flex items-center justify-center gap-2 transition-colors',
            canStartRecord
              ? 'bg-red-500 text-white hover:bg-red-600'
              : 'bg-gray-200 text-gray-400 cursor-not-allowed'
          )}
        >
          <MdFiberManualRecord size={18} />
          Record Start
        </button>
      </Tooltip>

      {/* SubTask count setup */}
      <div className="mb-3">
        <div className="text-sm font-semibold text-gray-700 mb-2">
          Number of SubTasks
          <span className="ml-1 text-xs font-normal text-gray-400">
            (plan sub_tasks in advance)
          </span>
        </div>
        <div className="flex items-center gap-2">
          <div
            className={clsx(
              'flex flex-1 items-stretch border border-gray-300 rounded-md overflow-hidden',
              {
                'bg-gray-100': isRecording || savingInProgress,
              }
            )}
          >
            <input
              type="number"
              min={minAllowedCount}
              max={MAX_PLANNED_SLOTS}
              value={plannedCount}
              onChange={(e) => handlePlanCountInput(e.target.value)}
              disabled={isRecording || savingInProgress}
              className={clsx(
                'flex-1 text-sm p-1.5 outline-none focus:ring-2 focus:ring-blue-500',
                '[appearance:textfield]',
                '[&::-webkit-outer-spin-button]:appearance-none',
                '[&::-webkit-inner-spin-button]:appearance-none',
                '[&::-webkit-inner-spin-button]:m-0',
                {
                  'bg-gray-100 cursor-not-allowed text-gray-500':
                    isRecording || savingInProgress,
                }
              )}
            />
            <div className="flex flex-col border-l border-gray-300">
              <button
                type="button"
                onClick={() => stepPlanCount(1)}
                disabled={
                  isRecording ||
                  savingInProgress ||
                  plannedCount >= MAX_PLANNED_SLOTS
                }
                className={clsx(
                  'flex-1 px-1 flex items-center justify-center',
                  'border-b border-gray-300 transition-colors',
                  isRecording ||
                    savingInProgress ||
                    plannedCount >= MAX_PLANNED_SLOTS
                    ? 'text-gray-300 cursor-not-allowed'
                    : 'text-gray-600 hover:bg-gray-100'
                )}
                aria-label="Increase subtask count"
                title="Add a slot"
              >
                <MdArrowDropUp size={18} />
              </button>
              <button
                type="button"
                onClick={() => stepPlanCount(-1)}
                disabled={
                  isRecording ||
                  savingInProgress ||
                  plannedCount <= minAllowedCount
                }
                className={clsx(
                  'flex-1 px-1 flex items-center justify-center',
                  'transition-colors',
                  isRecording ||
                    savingInProgress ||
                    plannedCount <= minAllowedCount
                    ? 'text-gray-300 cursor-not-allowed'
                    : 'text-gray-600 hover:bg-gray-100'
                )}
                aria-label="Decrease subtask count"
                title={
                  plannedCount <= minAllowedCount
                    ? 'Cannot drop below the number of saved segments'
                    : 'Remove the last slot'
                }
              >
                <MdArrowDropDown size={18} />
              </button>
            </div>
          </div>
          <button
            onClick={handleResetPlan}
            disabled={!canResetPlan}
            className={clsx(
              'px-3 py-1.5 rounded-md text-sm font-semibold transition-colors',
              canResetPlan
                ? 'bg-gray-300 text-gray-800 hover:bg-gray-400'
                : 'bg-gray-200 text-gray-400 cursor-not-allowed'
            )}
            title={
              numSavedInPlan > 0
                ? 'Cannot reset while segments are saved — discard them first'
                : 'Reset planned subtasks'
            }
          >
            Reset
          </button>
        </div>
      </div>

      {/* Slot rows */}
      <div className="flex flex-col gap-1.5 mb-3">
        {Array.from({ length: plannedCount }, (_, i) => renderSlotRow(i))}

        {isRecording && activeSlotIndex < plannedCount && (
          <div className="text-xs text-red-600 font-mono px-2">
            Recording slot #{activeSlotIndex + 1}: {plannedSubTasks[activeSlotIndex] || '—'} ({status.proceedTime}s)
          </div>
        )}
        {savingInProgress && (
          <div className="text-xs text-amber-700 font-mono px-2">
            Saving / advancing…
          </div>
        )}
        {planComplete && (
          <div className="text-xs text-indigo-700 font-mono px-2">
            All planned segments saved — press Finish Episode to finalize.
          </div>
        )}
        {!isPlanMode && (
          <div className="text-xs text-gray-400 italic px-2">
            Press “Add SubTask” to start planning your episode.
          </div>
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
