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

import React, { useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { setOutputFolderName, setModelWeightList } from '../features/training/trainingSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

export default function TrainingOutputFolderInput({ readonly = false }) {
  const dispatch = useDispatch();

  const { getModelWeightList } = useRosServiceCaller();

  const [checkingDuplicate, setCheckingDuplicate] = useState(false);
  const [duplicateChecked, setDuplicateChecked] = useState(false);
  const [isOutputFolderAvailable, setIsOutputFolderAvailable] = useState(false);
  const [tempOutputFolderName, setTempOutputFolderName] = useState('');

  const outputFolderName = useSelector((state) => state.training.trainingInfo.outputFolderName);

  const classCard = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-6',
    'w-full',
    'max-w-md',
    'min-w-[250px]'
  );

  const classInput = clsx(
    'w-full',
    'px-3',
    'py-2',
    'border',
    'border-gray-300',
    'rounded-md',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent'
  );

  const classButton = clsx(
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

  const classTitle = clsx('text-xl', 'font-bold', 'mb-6', 'text-left', {
    'text-gray-500': readonly,
    'text-gray-800': !readonly,
  });

  const handleCheckDuplicate = async () => {
    setCheckingDuplicate(true);
    console.log('check duplicate');

    try {
      const result = await getModelWeightList();

      if (result && result.model_weight_list) {
        dispatch(setModelWeightList(result.model_weight_list));
        console.log('result:', result);
        toast.success('Model weight list loaded successfully');
      } else {
        toast.error('Failed to get model weight list: Invalid response');
      }
      if (result.model_weight_list.includes(tempOutputFolderName)) {
        toast.error(`Output folder name "${tempOutputFolderName}" already exists`);
        setIsOutputFolderAvailable(false);
      } else {
        toast.success(`Output folder name "${tempOutputFolderName}" is available`);
        dispatch(setOutputFolderName(tempOutputFolderName));
        setIsOutputFolderAvailable(true);
      }
      setDuplicateChecked(true);
    } catch (error) {
      console.error('Error checking duplicate:', error);
      toast.error(`Failed to check duplicate: ${error.message}`);
    } finally {
      setCheckingDuplicate(false);
    }
  };

  useEffect(() => {
    if (outputFolderName) {
      console.log('outputFolderName:', outputFolderName);
      setTempOutputFolderName(outputFolderName);
    }
  }, [outputFolderName]);

  return (
    <div className={classCard}>
      <h1 className={classTitle}>Output folder name</h1>
      <div className="flex flex-row gap-2 items-center justify-center">
        {!duplicateChecked && !readonly && <p className="text-gray-500">Please check duplicate</p>}
        {duplicateChecked &&
          (isOutputFolderAvailable ? (
            <p className="text-blue-500 break-all">{tempOutputFolderName} is available</p>
          ) : (
            <p className="text-red-500 break-all">{tempOutputFolderName} already exists</p>
          ))}
      </div>
      <input
        type="text"
        className={classInput}
        disabled={checkingDuplicate || readonly}
        placeholder="Enter your text here..."
        value={tempOutputFolderName}
        onChange={(e) => {
          setTempOutputFolderName(e.target.value);
          dispatch(setOutputFolderName(undefined));
          setDuplicateChecked(false);
        }}
      />
      {!readonly && (
        <>
          <div className="mb-4" />
          <button
            className={classButton}
            onClick={handleCheckDuplicate}
            disabled={
              checkingDuplicate || duplicateChecked || tempOutputFolderName === '' || readonly
            }
          >
            Check duplicate
          </button>
        </>
      )}
    </div>
  );
}
