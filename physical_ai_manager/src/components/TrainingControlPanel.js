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

import React, { useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { setIsTraining, setTrainingMode, setLastUpdate } from '../features/training/trainingSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { DEFAULT_PATHS } from '../constants/paths';

export default function TrainingControlPanel() {
  const dispatch = useDispatch();
  const trainingMode = useSelector((state) => state.training.trainingMode);
  const isTraining = useSelector((state) => state.training.isTraining);
  const datasetRepoId = useSelector((state) => state.training.trainingInfo.datasetRepoId);
  const selectedPolicy = useSelector((state) => state.training.trainingInfo.policyType);
  const selectedDevice = useSelector((state) => state.training.trainingInfo.policyDevice);
  const outputFolderName = useSelector((state) => state.training.trainingInfo.outputFolderName);
  const resumePolicyPath = useSelector((state) => state.training.resumePolicyPath);
  const hasTrainConfig = useSelector((state) => state.training.hasTrainConfig);
  const isTrainingInfoLoaded = useSelector((state) => state.training.isTrainingInfoLoaded);
  const lastUpdate = useSelector((state) => state.training.lastUpdate);

  const { sendTrainingCommand } = useRosServiceCaller();

  const classContainer = clsx('flex', 'items-center', 'justify-center', 'p-2', 'gap-6', 'm-2');

  const classButton = clsx(
    'h-full',
    'px-8',
    'py-3',
    'rounded-2xl',
    'font-semibold',
    'text-lg',
    'transition-all',
    'duration-200',
    'transform',
    'active:scale-95',
    'shadow-lg'
  );

  const classStartButton = clsx(
    classButton,
    'bg-blue-600',
    'text-white',
    'hover:bg-blue-700',
    'hover:shadow-xl',
    'disabled:bg-gray-400',
    'disabled:cursor-not-allowed',
    'disabled:hover:bg-gray-400',
    'disabled:hover:shadow-lg'
  );

  const classFinishButton = clsx(
    classButton,
    'bg-red-600',
    'text-white',
    'hover:bg-red-700',
    'hover:shadow-xl',
    'disabled:bg-gray-400',
    'disabled:cursor-not-allowed',
    'disabled:hover:bg-gray-400',
    'disabled:hover:shadow-lg'
  );

  const classModeSelector = clsx('flex', 'items-center', 'gap-4', 'p-1');

  const classRadioGroup = clsx('flex', 'items-center', 'gap-2');

  const classRadioInput = clsx(
    'w-4',
    'h-4',
    'text-blue-600',
    'bg-gray-100',
    'border-gray-300',
    'focus:ring-blue-500',
    'focus:ring-2'
  );

  const classRadioLabel = clsx('text-lg', 'font-medium', 'text-gray-700', 'cursor-pointer');

  // Check if task status updates are paused (considered paused if no updates for 3 seconds)
  useEffect(() => {
    const UPDATE_PAUSE_THRESHOLD = 3000;
    const timer = setInterval(() => {
      const timeSinceLastUpdate = Date.now() - lastUpdate;
      const isPaused = timeSinceLastUpdate >= UPDATE_PAUSE_THRESHOLD;
      if (isPaused) {
        dispatch(setIsTraining(false));
      }
    }, 3000);

    return () => clearInterval(timer);
  }, [lastUpdate, dispatch]);

  const handleStartTraining = async () => {
    if (!checkRequiredFields()) {
      return;
    }

    try {
      let command;

      if (trainingMode === 'resume') {
        // Resume training
        if (!resumePolicyPath) {
          toast.error('Please select checkpoint path to resume training');
          return;
        }
        command = 'resume'; // RESUME
      } else {
        // New training
        if (!datasetRepoId || !selectedPolicy || !selectedDevice || !outputFolderName) {
          toast.error('Please fill in all required fields');
          return;
        }
        command = 'start'; // START
      }

      const result = await sendTrainingCommand(command);

      if (result.success) {
        toast.success(
          trainingMode === 'resume'
            ? 'Training resumed successfully!'
            : 'Training started successfully!'
        );
        dispatch(setIsTraining(true));
        dispatch(setLastUpdate(Date.now()));
      } else {
        toast.error(
          `Failed to ${trainingMode === 'resume' ? 'resume' : 'start'} training: ${result.message}`
        );
        dispatch(setIsTraining(false));
      }
    } catch (error) {
      toast.error(
        `Error ${trainingMode === 'resume' ? 'resuming' : 'starting'} training: ${error.message}`
      );
      dispatch(setIsTraining(false));
    }
  };

  const handleFinishTraining = async () => {
    try {
      const result = await sendTrainingCommand('finish');

      if (result.success) {
        toast.success('Training finished successfully!');
        dispatch(setIsTraining(false));
      } else {
        toast.error(`Failed to finish training: ${result.message}`);
        dispatch(setIsTraining(true));
      }
    } catch (error) {
      toast.error(`Error finishing training: ${error.message}`);
      dispatch(setIsTraining(true));
    }
  };

  const getStartButtonText = () => {
    if (trainingMode === 'resume') {
      return 'Resume Training';
    }
    return 'Start Training';
  };

  const checkRequiredFields = () => {
    if (trainingMode === 'resume') {
      if (!resumePolicyPath) {
        toast.error('Please select checkpoint path to resume training');
        return false;
      }
      if (!resumePolicyPath.startsWith(DEFAULT_PATHS.POLICY_MODEL_PATH)) {
        toast.error(`Policy path must be under: ${DEFAULT_PATHS.POLICY_MODEL_PATH}`);
        return false;
      }
      if (hasTrainConfig === false) {
        toast.error('train_config.json file not found in selected path');
        return false;
      }
      if (hasTrainConfig === null) {
        toast.error('Please wait for path validation to complete');
        return false;
      }
      if (!isTrainingInfoLoaded) {
        toast.error('Please press the Load button to load training info first', {
          duration: 4000,
        });
        return false;
      }
      return true;
    } else {
      if (!datasetRepoId) {
        toast.error('Please select a dataset repository');
        return false;
      }
      if (!selectedPolicy) {
        toast.error('Please select a policy');
        return false;
      }
      if (!selectedDevice) {
        toast.error('Please select a device');
        return false;
      }
      if (!outputFolderName) {
        toast.error('Please select an output folder and check if it is not duplicated');
        return false;
      }
      return true;
    }
  };

  const handleModeChange = (mode) => {
    dispatch(setTrainingMode(mode));
  };

  return (
    <div className={classContainer}>
      {/* Training Mode Selector */}
      <div className={classModeSelector}>
        <h3 className="text-xl font-bold text-gray-800 mr-4">Training Mode</h3>

        <div className="flex flex-col items-start gap-2">
          <div className={classRadioGroup}>
            <input
              type="radio"
              id="new-training"
              name="trainingMode"
              value="new"
              checked={trainingMode === 'new'}
              onChange={() => handleModeChange('new')}
              className={classRadioInput}
              disabled={isTraining}
            />
            <label htmlFor="new-training" className={classRadioLabel}>
              New Training
            </label>
          </div>

          <div className={classRadioGroup}>
            <input
              type="radio"
              id="resume-training"
              name="trainingMode"
              value="resume"
              checked={trainingMode === 'resume'}
              onChange={() => handleModeChange('resume')}
              className={classRadioInput}
              disabled={false}
            />
            <label htmlFor="resume-training" className={clsx(classRadioLabel, 'text-gray-500')}>
              Resume Training
            </label>
          </div>
        </div>
      </div>
      <button
        onClick={handleStartTraining}
        className={classStartButton}
        style={{ display: isTraining ? 'none' : 'block' }}
      >
        {getStartButtonText()}
      </button>

      <button
        onClick={handleFinishTraining}
        disabled={!isTraining}
        className={classFinishButton}
        style={{ display: isTraining ? 'block' : 'none' }}
      >
        Finish Training
      </button>
    </div>
  );
}
