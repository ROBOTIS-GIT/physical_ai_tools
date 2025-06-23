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

// Flux imports
import { useAppStore } from '../flux/hooks/useAppStore';
import { useTaskStore } from '../flux/hooks/useTaskStore';
import AppActions from '../flux/actions/AppActions';

export default function RobotTypeSelector({ className }) {
  // Get state from Flux stores
  const { rosHost, currentRobotType } = useAppStore();
  const { taskStatus } = useTaskStore();

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
        // Update robot type in the store
        AppActions.setRobotType(selectedRobotType);

        // Update task status with the selected robot type
        AppActions.updateTaskStatus({ robotType: selectedRobotType });

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
    if (currentRobotType) {
      AppActions.updateTaskStatus({ robotType: currentRobotType });
    }
  }, [currentRobotType]);

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
    'mb-4',
    'text-center'
  );

  return (
    <div className={classCard}>
      <h2 className={classTitle}>Robot Type Selection</h2>

      <div>
        <label className={classLabel}>Available Robot Types:</label>
        <select
          value={selectedRobotType}
          onChange={(e) => setSelectedRobotType(e.target.value)}
          className={classSelect}
          disabled={loading || fetching}
        >
          <option value="">Select a robot type</option>
          {robotTypes.map((type) => (
            <option key={type} value={type}>
              {type}
            </option>
          ))}
        </select>
      </div>

      <button
        onClick={handleSetRobotType}
        disabled={loading || !selectedRobotType || fetching}
        className={classButton}
      >
        {loading ? 'Setting...' : 'Set Robot Type'}
      </button>

      <button
        onClick={fetchRobotTypes}
        disabled={fetching || loading}
        className={classRefreshButton}
      >
        <MdRefresh className="inline mr-2" />
        {fetching ? 'Refreshing...' : 'Refresh List'}
      </button>

      {currentRobotType && (
        <div className={classCurrentType}>
          <strong>Current Robot Type:</strong> {currentRobotType}
        </div>
      )}

      {taskStatus && taskStatus.robotType && (
        <div className={classCurrentType}>
          <strong>Task Robot Type:</strong> {taskStatus.robotType}
        </div>
      )}
    </div>
  );
}
