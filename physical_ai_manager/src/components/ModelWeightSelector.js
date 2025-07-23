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
import { MdRefresh, MdFolder, MdCheckCircle } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { setModelWeightList, setSelectedModelWeight } from '../features/training/trainingSlice';

export default function ModelWeightSelector() {
  const dispatch = useDispatch();

  const modelWeightList = useSelector((state) => state.training.modelWeightList);
  const selectedModelWeight = useSelector((state) => state.training.selectedModelWeight);
  const isTraining = useSelector((state) => state.training.isTraining);

  const { getModelWeightList } = useRosServiceCaller();

  const [loading, setLoading] = useState(false);

  // Fetch model weight list
  const fetchModelWeights = useCallback(async () => {
    setLoading(true);
    try {
      const result = await getModelWeightList();
      console.log('Model weights received:', result);

      if (result && result.model_weight_list) {
        dispatch(setModelWeightList(result.model_weight_list));
        toast.success('Model weight list loaded successfully');
      } else {
        toast.error('Failed to get model weight list: Invalid response');
      }
    } catch (error) {
      console.error('Error fetching model weights:', error);
      toast.error(`Failed to get model weight list: ${error.message}`);
    } finally {
      setLoading(false);
    }
  }, [getModelWeightList, dispatch]);

  // Handle model weight selection
  const handleModelWeightSelection = useCallback(
    (modelWeightPath) => {
      dispatch(setSelectedModelWeight(modelWeightPath));
      toast.success(`Model weight selected: ${modelWeightPath}`);
    },
    [dispatch]
  );

  // Fetch model weights when component mounts
  useEffect(() => {
    fetchModelWeights();
  }, [fetchModelWeights]);

  const classCard = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-8',
    'w-full',
    'max-w-lg'
  );

  const classTitle = clsx('text-2xl', 'font-bold', 'text-gray-800', 'mb-6', 'text-center');

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
    'disabled:cursor-not-allowed',
    'flex',
    'items-center',
    'justify-center',
    'gap-2',
    'mb-4'
  );

  const classCurrentSelection = clsx(
    'text-sm',
    'text-gray-600',
    'bg-gray-100',
    'px-3',
    'py-2',
    'rounded-md',
    'text-center',
    'mb-4'
  );

  const classListContainer = clsx(
    'border',
    'border-gray-300',
    'rounded-md',
    'max-h-96',
    'overflow-y-auto',
    'bg-gray-50'
  );

  const classModelWeightItem = (isSelected) =>
    clsx(
      'flex',
      'items-center',
      'px-3',
      'py-2',
      'cursor-pointer',
      'transition-colors',
      'border-b',
      'border-gray-200',
      {
        'bg-blue-50 hover:bg-blue-100': isSelected,
        'hover:bg-gray-100': !isSelected,
      }
    );

  const classModelWeightIcon = (isSelected) =>
    clsx('w-5', 'h-5', 'mr-3', {
      'text-blue-600': isSelected,
      'text-gray-600': !isSelected,
    });

  const classModelWeightName = (isSelected) =>
    clsx('text-sm', 'flex-1', {
      'text-blue-700 font-medium': isSelected,
      'text-gray-700': !isSelected,
    });

  const classSelectedIcon = clsx('w-5', 'h-5', 'mr-3', 'text-blue-600');

  return (
    <div className="flex flex-col items-center justify-center">
      <div className={classCard}>
        <div className={classTitle}>Model Weights</div>

        {selectedModelWeight && (
          <div className={classCurrentSelection}>
            <b>Selected:</b> {selectedModelWeight}
          </div>
        )}

        <button
          onClick={fetchModelWeights}
          disabled={loading || isTraining}
          className={classRefreshButton}
        >
          <MdRefresh className={clsx('w-4', 'h-4', { 'animate-spin': loading })} />
          {loading ? 'Loading...' : 'Refresh Model Weights'}
        </button>

        <div className={classListContainer}>
          {modelWeightList && modelWeightList.length > 0 ? (
            modelWeightList.map((modelWeightPath) => {
              const isSelected = selectedModelWeight === modelWeightPath;

              return (
                <div
                  key={modelWeightPath}
                  onClick={() => !isTraining && handleModelWeightSelection(modelWeightPath)}
                  className={classModelWeightItem(isSelected)}
                >
                  <MdFolder className={classModelWeightIcon(isSelected)} />
                  <span className={classModelWeightName(isSelected)}>{modelWeightPath}</span>
                  {isSelected && <MdCheckCircle className={classSelectedIcon} />}
                </div>
              );
            })
          ) : (
            <div className="text-center text-gray-500 py-8">
              {loading ? 'Loading model weights...' : 'No model weights available'}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
