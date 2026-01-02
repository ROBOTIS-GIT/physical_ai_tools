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

import React, { useCallback, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdRefresh } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import TaskPhase from '../constants/taskPhases';
import { selectRobotType, removeAllTags } from '../features/tasks/taskSlice';
import { setRobotTypeList, setIsFirstLoadTrue } from '../features/ui/uiSlice';

export default function RobotTypeSelector() {
  const dispatch = useDispatch();

  const robotTypeList = useSelector((state) => state.ui.robotTypeList);
  const robotType = useSelector((state) => state.tasks.taskStatus.robotType);
  const taskStatus = useSelector((state) => state.tasks.taskStatus);

  const { getRobotTypeList, setRobotType } = useRosServiceCaller();

  const [loading, setLoading] = useState(false);
  const [fetching, setFetching] = useState(false);
  const [selectedRobotType, setSelectedRobotType] = useState('');

  // Fetch robot type list
  const fetchRobotTypes = useCallback(async () => {
    setFetching(true);
    try {
      const result = await getRobotTypeList();
      console.log('Robot types received:', result);

      if (result && result.robot_types) {
        dispatch(setRobotTypeList(result.robot_types));
        toast.success('Robot types loaded successfully');
      } else {
        toast.error('Failed to get robot types: Invalid response');
      }
    } catch (error) {
      console.error('Error fetching robot types:', error);
      toast.error(`Failed to get robot types: ${error.message}`);
    } finally {
      setFetching(false);
    }
  }, [getRobotTypeList, dispatch]);

  const handleSetRobotType = async () => {
    console.log('handleSetRobotType called');
    console.log('selectedRobotType:', selectedRobotType);

    if (!selectedRobotType) {
      toast.error('Please select a robot type');
      return;
    }

    // Prevent changing robot type while task is in progress
    if (taskStatus.phase > TaskPhase.READY) {
      toast.error('Cannot change robot type while task is in progress', {
        duration: 4000,
      });
      return;
    }

    console.log('Attempting to set robot type to:', selectedRobotType);
    setLoading(true);
    try {
      const result = await setRobotType(selectedRobotType);
      console.log('Set robot type result:', result);

      if (result && result.success) {
        dispatch(selectRobotType(selectedRobotType));
        toast.success(`Robot type set to: ${selectedRobotType}`);

        dispatch(setIsFirstLoadTrue('record')); // to reset tags
        dispatch(removeAllTags());
      } else {
        toast.error(`Failed to set robot type: ${result.message || 'Unknown error'}`);
      }
    } catch (error) {
      console.error('Error setting robot type:', error);
      toast.error(`Failed to set robot type: ${error.message}`);
    } finally {
      setLoading(false);
    }
  };

  // Fetch robot types when component mounts
  useEffect(() => {
    fetchRobotTypes();
  }, [fetchRobotTypes]);

  useEffect(() => {
    if (robotType && !selectedRobotType) {
      setSelectedRobotType(robotType);
    }
  }, [robotType, selectedRobotType]);

  const classCard = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-8',
    'w-full',
    'max-w-md'
  );

  const classTitle = clsx('text-2xl', 'font-bold', 'text-gray-800', 'mb-6', 'text-center');
  const classLabel = clsx('text-sm', 'font-medium', 'text-gray-700', 'mb-2', 'block');
  const classSelect = clsx(
    'w-full',
    'px-3',
    'py-2',
    'border',
    'border-gray-300',
    'rounded-md',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent',
    'mb-4'
  );

  const classButton = clsx(
    'w-full',
    'px-4',
    'py-2',
    'bg-blue-500',
    'text-white',
    'rounded-md',
    'font-medium',
    'transition-colors',
    'hover:bg-blue-600',
    'disabled:bg-gray-400',
    'disabled:cursor-not-allowed',
    'mb-3'
  );

  const classRefreshButton = clsx(
    'w-full',
    'px-4',
    'py-2',
    'bg-gray-500',
    'text-white',
    'rounded-md',
    'font-medium',
    'transition-colors',
    'hover:bg-gray-600',
    'disabled:bg-gray-400',
    'disabled:cursor-not-allowed'
  );

  const classCurrentType = clsx(
    'text-sm',
    'text-gray-600',
    'bg-gray-100',
    'px-3',
    'py-2',
    'rounded-md',
    'text-center',
    'mb-4'
  );

  return (
    <div className={classCard}>
      <h1 className={classTitle}>Robot Type Selection</h1>

      {robotType && (
        <div className={classCurrentType}>
          <strong>Current Robot Type:</strong> {robotType}
          {taskStatus.robotType && (
            <div className="text-xs text-green-600 mt-1">
              ✓ Saved to Task Status: {taskStatus.robotType}
            </div>
          )}
        </div>
      )}

      {taskStatus.phase > 0 && (
        <div className="text-sm text-orange-600 bg-orange-100 px-3 py-2 rounded-md text-center mb-4">
          <strong>⚠️ Task in progress (Phase {taskStatus.phase})</strong>
          <div className="text-xs mt-1">Robot type cannot be changed during task execution</div>
        </div>
      )}

      <label className={classLabel}>Select Robot Type:</label>

      <select
        className={classSelect}
        value={selectedRobotType}
        onChange={(e) => setSelectedRobotType(e.target.value)}
        disabled={fetching || loading || taskStatus.phase > 0}
      >
        <option value="" disabled>
          Choose a robot type...
        </option>
        {robotTypeList.map((type) => (
          <option key={type} value={type}>
            {type}
          </option>
        ))}
      </select>

      <button
        className={classButton}
        onClick={handleSetRobotType}
        disabled={loading || fetching || !selectedRobotType || taskStatus.phase > 0}
      >
        {loading ? 'Setting...' : 'Set Robot Type'}
      </button>

      <button
        className={classRefreshButton}
        onClick={fetchRobotTypes}
        disabled={fetching || loading || taskStatus.phase > 0}
      >
        <div className="flex items-center justify-center gap-2">
          <MdRefresh size={16} className={fetching ? 'animate-spin' : ''} />
          {fetching ? 'Loading...' : 'Refresh Robot Type List'}
        </div>
      </button>

      {robotTypeList.length === 0 && !fetching && (
        <div className="text-center text-gray-500 text-sm mt-4">
          No robot types available. Please check ROS connection.
        </div>
      )}
    </div>
  );
}
