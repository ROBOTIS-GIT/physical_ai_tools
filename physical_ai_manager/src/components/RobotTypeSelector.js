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

import React, { useState, useEffect, useCallback } from 'react';
import clsx from 'clsx';
import { MdRefresh } from 'react-icons/md';
import toast from 'react-hot-toast';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

export default function RobotTypeSelector({
  rosHost,
  currentRobotType,
  setCurrentRobotType,
  taskStatus,
  updateTaskStatus,
  className,
}) {
  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;
  const { getRobotTypeList, setRobotType } = useRosServiceCaller(rosbridgeUrl);

  const [robotTypes, setRobotTypes] = useState([]);
  const [selectedRobotType, setSelectedRobotType] = useState(currentRobotType || '');
  const [loading, setLoading] = useState(false);
  const [fetching, setFetching] = useState(false);

  // Fetch robot type list
  const fetchRobotTypes = useCallback(async () => {
    setFetching(true);
    try {
      const result = await getRobotTypeList();
      console.log('Robot types received:', result);
      console.log('Current currentRobotType before update:', currentRobotType);

      if (result && result.robot_types) {
        setRobotTypes(result.robot_types);
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
  }, [getRobotTypeList, currentRobotType]);

  useEffect(() => {
    if (!selectedRobotType && taskStatus && taskStatus.robotType) {
      setSelectedRobotType(taskStatus.robotType);
    }
  }, [taskStatus, selectedRobotType]);

  // Set robot type
  const handleSetRobotType = async () => {
    console.log('handleSetRobotType called');
    console.log('selectedRobotType:', selectedRobotType);
    console.log('currentRobotType:', currentRobotType);

    if (!selectedRobotType) {
      toast.error('Please select a robot type');
      return;
    }

    // Only prevent setting if currentRobotType exists and matches selectedRobotType
    if (currentRobotType && selectedRobotType === currentRobotType) {
      console.log('Robot type already set, skipping');
      toast.error('Robot type is already set to this value');
      return;
    }

    // Prevent changing robot type while task is in progress
    if (taskStatus && taskStatus.phase > 0) {
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
        setCurrentRobotType(selectedRobotType);
        // Update task status with the selected robot type
        if (updateTaskStatus) {
          updateTaskStatus({ robotType: selectedRobotType });
        }
        toast.success(`Robot type set to: ${selectedRobotType}`);
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

  // Sync selectedRobotType when currentRobotType changes
  useEffect(() => {
    if (!selectedRobotType && currentRobotType) {
      setSelectedRobotType(currentRobotType);
    }
  }, [currentRobotType, selectedRobotType]);

  // Initialize task info robot type if currentRobotType is set but task info is empty
  useEffect(() => {
    if (currentRobotType && updateTaskStatus) {
      updateTaskStatus({ robotType: currentRobotType });
    }
  }, [currentRobotType, updateTaskStatus]);

  const classCard = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-8',
    'w-full',
    'max-w-md',
    className
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

      {currentRobotType && (
        <div className={classCurrentType}>
          <strong>Current Robot Type:</strong> {currentRobotType}
          {taskStatus && taskStatus.robotType && (
            <div className="text-xs text-green-600 mt-1">
              ✓ Saved to Task Status: {taskStatus.robotType}
            </div>
          )}
        </div>
      )}

      {taskStatus && taskStatus.phase > 0 && (
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
        disabled={fetching || loading || (taskStatus && taskStatus.phase > 0)}
      >
        <option value="" disabled>
          Choose a robot type...
        </option>
        {robotTypes.map((type) => (
          <option key={type} value={type}>
            {type}
          </option>
        ))}
      </select>

      <button
        className={classButton}
        onClick={handleSetRobotType}
        disabled={loading || fetching || !selectedRobotType || (taskStatus && taskStatus.phase > 0)}
      >
        {loading ? 'Setting...' : 'Set Robot Type'}
      </button>

      <button
        className={classRefreshButton}
        onClick={fetchRobotTypes}
        disabled={fetching || loading || (taskStatus && taskStatus.phase > 0)}
      >
        <div className="flex items-center justify-center gap-2">
          <MdRefresh size={16} className={fetching ? 'animate-spin' : ''} />
          {fetching ? 'Loading...' : 'Refresh Robot Type List'}
        </div>
      </button>

      {robotTypes.length === 0 && !fetching && (
        <div className="text-center text-gray-500 text-sm mt-4">
          No robot types available. Please check ROS connection.
        </div>
      )}
    </div>
  );
}
