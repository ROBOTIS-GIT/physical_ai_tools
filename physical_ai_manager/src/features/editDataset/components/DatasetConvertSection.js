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

import React, { useCallback, useEffect, useState } from 'react';
import clsx from 'clsx';
import { useDispatch, useSelector } from 'react-redux';
import { MdFolderOpen, MdMovie } from 'react-icons/md';
import FileBrowserModal from '../../../components/FileBrowserModal';
import TaskPhase from '../../../constants/taskPhases';
import { setTaskInfo } from '../../../features/tasks/taskSlice';
import { useRosServiceCaller } from '../../../hooks/useRosServiceCaller';
import { DEFAULT_PATHS } from '../../../constants/paths';

// Convert a task folder name like "ffw_sg2_rev1_test_0406_1" to the
// task_name expected by the backend (strip the "{robotType}_" prefix when
// it matches). The backend rebuilds the full path as
// /workspace/rosbag2/{robot_type}_{task_name}.
const stripRobotPrefix = (folderName, robotType) => {
  if (!robotType) return folderName;
  const prefix = `${robotType}_`;
  return folderName.startsWith(prefix) ? folderName.slice(prefix.length) : folderName;
};

export default function DatasetConvertSection({ isEditable = true }) {
  const dispatch = useDispatch();
  const { sendRecordCommand } = useRosServiceCaller();
  const info = useSelector((state) => state.tasks.taskInfo);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const robotType = taskStatus?.robotType || '';

  // ----- local state ------------------------------------------------------
  const [singleTaskName, setSingleTaskName] = useState('');
  const [showSingleBrowser, setShowSingleBrowser] = useState(false);

  const [mergeMode, setMergeMode] = useState(false);
  const [sourceFolders, setSourceFolders] = useState([]);
  const [mergedName, setMergedName] = useState('');
  const [showSourceBrowser, setShowSourceBrowser] = useState(false);

  const [isConverting, setIsConverting] = useState(false);
  const [hasSeenConverting, setHasSeenConverting] = useState(false);
  const [convertError, setConvertError] = useState('');
  const [pendingMergeConvert, setPendingMergeConvert] = useState(false);
  const [pendingSingleConvert, setPendingSingleConvert] = useState(false);

  // ----- isConverting tracks the backend phase ----------------------------
  useEffect(() => {
    if (taskStatus.phase === TaskPhase.CONVERTING) {
      setIsConverting(true);
      setHasSeenConverting(true);
    } else if (
      isConverting &&
      hasSeenConverting &&
      taskStatus.phase === TaskPhase.READY
    ) {
      setIsConverting(false);
      setHasSeenConverting(false);
    }
  }, [taskStatus.phase, isConverting, hasSeenConverting]);

  // ----- single convert ---------------------------------------------------
  // Same two-step pattern as merge: dispatch taskInfo first, then fire
  // sendRecordCommand from a useEffect once Redux has propagated the update
  // (sendRecordCommand reads taskInfo via closure, so it must see the new
  // value before being invoked).
  const handleConvertMp4 = useCallback(() => {
    if (!singleTaskName) {
      setConvertError('Please pick a task folder to convert');
      return;
    }
    setConvertError('');
    setIsConverting(true);

    dispatch(
      setTaskInfo({
        ...info,
        taskName: singleTaskName,
        taskInstruction: [singleTaskName],
      })
    );
    setPendingSingleConvert(true);
  }, [singleTaskName, info, dispatch]);

  useEffect(() => {
    if (!pendingSingleConvert) return;
    if (info.taskName !== singleTaskName) return;
    setPendingSingleConvert(false);

    const fire = async () => {
      try {
        const result = await sendRecordCommand('convert_mp4');
        if (!result?.success) {
          setConvertError(result?.message || 'Conversion failed');
          setIsConverting(false);
        }
      } catch (error) {
        setConvertError(error.message || 'Failed to start conversion');
        setIsConverting(false);
      }
    };
    fire();
  }, [pendingSingleConvert, info.taskName, singleTaskName, sendRecordCommand]);

  // ----- merge & convert --------------------------------------------------
  // Same two-step pattern as the original InfoPanel.js handler: dispatch
  // setTaskInfo, then fire sendRecordCommand from a useEffect once Redux
  // has propagated the new taskInfo.
  const handleMergeConvert = useCallback(() => {
    if (!mergedName) {
      setConvertError('Merged output name is required');
      return;
    }
    if (sourceFolders.length < 2) {
      setConvertError('At least 2 source folders are required');
      return;
    }
    setConvertError('');
    setIsConverting(true);

    dispatch(
      setTaskInfo({
        ...info,
        taskName: mergedName,
        taskInstruction: sourceFolders,
      })
    );
    setPendingMergeConvert(true);
  }, [mergedName, sourceFolders, info, dispatch]);

  useEffect(() => {
    if (!pendingMergeConvert) return;
    if (info.taskName !== mergedName) return;
    setPendingMergeConvert(false);

    const fire = async () => {
      try {
        const result = await sendRecordCommand('convert_mp4');
        if (!result?.success) {
          setConvertError(result?.message || 'Merge & Convert failed');
          setIsConverting(false);
        }
      } catch (error) {
        setConvertError(error.message || 'Failed to start Merge & Convert');
        setIsConverting(false);
      }
    };
    fire();
  }, [pendingMergeConvert, info.taskName, mergedName, sendRecordCommand]);

  // ----- file browser callbacks ------------------------------------------
  const handleSingleFolderSelect = useCallback(
    (item) => {
      const taskName = stripRobotPrefix(item.name, robotType);
      setSingleTaskName(taskName);
      setShowSingleBrowser(false);
    },
    [robotType]
  );

  const handleSourceFoldersSelect = useCallback((selectedItems) => {
    const items = Array.isArray(selectedItems) ? selectedItems : [selectedItems];
    setSourceFolders((prev) => {
      const next = new Set(prev);
      items.forEach((it) => next.add(it.name));
      return Array.from(next);
    });
  }, []);

  // ----- derived ----------------------------------------------------------
  const canConvertSingle =
    !isConverting && !mergeMode && Boolean(singleTaskName) && isEditable;
  const canConvertMerge =
    !isConverting && mergeMode && sourceFolders.length >= 2 && Boolean(mergedName) && isEditable;

  return (
    <div className="w-full flex flex-col items-center justify-start bg-gray-100 p-10 gap-8 rounded-xl">
      <div className="w-full flex items-center justify-start gap-2">
        <MdMovie className="w-7 h-7 text-blue-500" />
        <span className="text-2xl font-bold">Convert Dataset</span>
      </div>

      <div className="w-full bg-white p-6 rounded-md shadow-md flex flex-col gap-4">
        {/* Single convert -------------------------------------------------- */}
        {!mergeMode && (
          <div className="flex flex-col gap-2">
            <span className="text-sm text-gray-600 font-medium">
              Task folder to convert
            </span>
            <div className="flex flex-row items-center gap-2">
              <input
                type="text"
                value={singleTaskName}
                onChange={(e) => setSingleTaskName(e.target.value)}
                placeholder="e.g. test_0406_1"
                disabled={isConverting || !isEditable}
                className={clsx(
                  'text-sm flex-1 p-2 border border-gray-300 rounded-md',
                  'focus:outline-none focus:ring-2 focus:ring-blue-500',
                  (isConverting || !isEditable) && 'bg-gray-100 cursor-not-allowed'
                )}
              />
              <button
                type="button"
                onClick={() => setShowSingleBrowser(true)}
                disabled={isConverting || !isEditable}
                className="flex items-center justify-center w-10 h-10 text-blue-500 bg-gray-200 rounded-md hover:text-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
                aria-label="Browse for task folder"
              >
                <MdFolderOpen className="w-6 h-6" />
              </button>
            </div>
            <div className="text-xs text-gray-500">
              Picks a folder under <code>/workspace/rosbag2/</code>. The
              <code> {robotType ? `${robotType}_` : ''}</code> prefix is added
              automatically when calling the converter.
            </div>

            <button
              type="button"
              onClick={handleConvertMp4}
              disabled={!canConvertSingle}
              className={clsx(
                'mt-2 px-4 py-2 text-sm font-medium rounded-lg transition-colors flex items-center justify-center gap-2',
                canConvertSingle
                  ? 'bg-green-500 text-white hover:bg-green-600'
                  : 'bg-gray-300 text-gray-500 cursor-not-allowed'
              )}
            >
              {isConverting && !mergeMode ? 'Converting…' : 'Convert Dataset'}
            </button>
          </div>
        )}

        {/* Mode toggle ----------------------------------------------------- */}
        <label className="flex items-center gap-2 cursor-pointer mt-2">
          <input
            type="checkbox"
            checked={mergeMode}
            onChange={(e) => setMergeMode(e.target.checked)}
            disabled={isConverting}
            className="rounded"
          />
          <span className="text-sm text-gray-700 font-medium">Merge &amp; Convert</span>
        </label>

        {/* Merge & Convert UI --------------------------------------------- */}
        {mergeMode && (
          <div className="w-full mt-2 p-3 bg-gray-50 rounded-lg border border-gray-200 flex flex-col gap-2">
            <div className="text-xs text-gray-600 font-medium">Source folders:</div>
            {sourceFolders.length === 0 ? (
              <div className="text-xs text-gray-400">No folders added yet</div>
            ) : (
              <div className="space-y-1">
                {sourceFolders.map((folder, idx) => (
                  <div
                    key={`${folder}-${idx}`}
                    className="flex items-center gap-1 text-xs bg-white rounded px-2 py-1 border border-gray-200"
                  >
                    <span className="text-gray-500 font-mono w-4">{idx}.</span>
                    <span className="flex-1 truncate text-gray-700" title={folder}>
                      {folder}
                    </span>
                    <button
                      type="button"
                      onClick={() =>
                        setSourceFolders((prev) => prev.filter((_, i) => i !== idx))
                      }
                      disabled={isConverting}
                      className="text-red-400 hover:text-red-600 font-bold ml-1 disabled:opacity-50"
                    >
                      x
                    </button>
                  </div>
                ))}
              </div>
            )}
            <button
              type="button"
              onClick={() => setShowSourceBrowser(true)}
              disabled={isConverting}
              className={clsx(
                'self-start px-3 py-1 text-xs font-medium rounded-md flex items-center gap-1 transition-colors',
                isConverting
                  ? 'bg-gray-300 text-gray-500 cursor-not-allowed'
                  : 'bg-blue-500 text-white hover:bg-blue-600'
              )}
            >
              <MdFolderOpen size={14} />
              Add folder
            </button>

            <div className="mt-2">
              <div className="text-xs text-gray-600 font-medium mb-1">
                Merged output name:
              </div>
              <input
                type="text"
                value={mergedName}
                onChange={(e) => setMergedName(e.target.value)}
                disabled={isConverting}
                placeholder="e.g. recycle_task"
                className={clsx(
                  'text-sm w-full p-1.5 border border-gray-300 rounded-md',
                  'focus:outline-none focus:ring-2 focus:ring-blue-500',
                  isConverting && 'bg-gray-100 cursor-not-allowed'
                )}
              />
            </div>

            <button
              type="button"
              onClick={handleMergeConvert}
              disabled={!canConvertMerge}
              className={clsx(
                'mt-2 px-4 py-2 text-sm font-medium rounded-lg flex items-center justify-center gap-2 transition-colors',
                canConvertMerge
                  ? 'bg-green-500 text-white hover:bg-green-600'
                  : 'bg-gray-300 text-gray-500 cursor-not-allowed'
              )}
            >
              {isConverting && mergeMode ? 'Merging & Converting…' : 'Merge & Convert'}
            </button>
          </div>
        )}

        {/* Progress -------------------------------------------------------- */}
        {isConverting && taskStatus.phase === TaskPhase.CONVERTING && (
          <div className="w-full mt-2">
            <div className="flex justify-between text-xs text-gray-600 mb-1">
              <span>
                {mergeMode
                  ? (taskStatus.encodingProgress || 0) < 5
                    ? '[0/3] Merging episodes…'
                    : (taskStatus.encodingProgress || 0) < 35
                      ? '[1/3] Converting to MP4…'
                      : (taskStatus.encodingProgress || 0) < 68
                        ? '[2/3] Converting to LeRobot v2.1…'
                        : '[3/3] Converting to LeRobot v3.0…'
                  : (taskStatus.encodingProgress || 0) < 33
                    ? '[1/3] Converting to MP4…'
                    : (taskStatus.encodingProgress || 0) < 66
                      ? '[2/3] Converting to LeRobot v2.1…'
                      : '[3/3] Converting to LeRobot v3.0…'}
              </span>
              <span>{Math.round(taskStatus.encodingProgress || 0)}%</span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div
                className="bg-green-500 h-2 rounded-full transition-all duration-300"
                style={{ width: `${taskStatus.encodingProgress || 0}%` }}
              />
            </div>
          </div>
        )}

        {convertError && (
          <div className="text-xs text-red-500 mt-1">{convertError}</div>
        )}
        <div className="text-xs text-gray-500 mt-1 leading-relaxed">
          Convert to MP4, LeRobot v2.1, and v3.0 formats.
        </div>
      </div>

      {/* File browsers ---------------------------------------------------- */}
      <FileBrowserModal
        isOpen={showSingleBrowser}
        onClose={() => setShowSingleBrowser(false)}
        onFileSelect={handleSingleFolderSelect}
        title="Select task folder to convert"
        selectButtonText="Select"
        allowDirectorySelect={true}
        allowFileSelect={false}
        initialPath={DEFAULT_PATHS.ROSBAG2_PATH}
        defaultPath={DEFAULT_PATHS.ROSBAG2_PATH}
        homePath=""
      />
      <FileBrowserModal
        isOpen={showSourceBrowser}
        onClose={() => setShowSourceBrowser(false)}
        onFileSelect={handleSourceFoldersSelect}
        title="Select source folders to merge"
        selectButtonText="Select"
        allowDirectorySelect={true}
        allowFileSelect={false}
        initialPath={DEFAULT_PATHS.ROSBAG2_PATH}
        defaultPath={DEFAULT_PATHS.ROSBAG2_PATH}
        homePath=""
        multiSelect={true}
      />
    </div>
  );
}
