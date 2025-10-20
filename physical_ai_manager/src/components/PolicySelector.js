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
import { useSelector, useDispatch } from 'react-redux';
import {
  selectPolicyType,
  selectPolicyDevice,
  setPolicyList,
  setDeviceList,
} from '../features/training/trainingSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import toast from 'react-hot-toast';

export default function PolicySelector({ readonly = false }) {
  const dispatch = useDispatch();

  const selectedPolicy = useSelector((state) => state.training.trainingInfo.policyType);
  const selectedDevice = useSelector((state) => state.training.trainingInfo.policyDevice);
  const policyList = useSelector((state) => state.training.policyList);
  const deviceList = useSelector((state) => state.training.deviceList);
  const isTraining = useSelector((state) => state.training.isTraining);

  const title = 'Policy Selection';

  const [loading] = useState(false);
  const [fetching, setFetching] = useState(false);

  const { getPolicyList } = useRosServiceCaller();

  const fetchItemList = useCallback(async () => {
    setFetching(true);
    try {
      const result = await getPolicyList();
      console.log('Policies received:', result);
      if (result && result.policy_list) {
        dispatch(setPolicyList(result.policy_list));
        dispatch(setDeviceList(result.device_list));
        toast.success('Policy list loaded successfully');
      } else {
        toast.error('Failed to get policy list: Invalid response');
      }
    } catch (error) {
      console.error('Error fetching policy list:', error);
    } finally {
      setFetching(false);
    }
  }, [getPolicyList, dispatch]);

  const classCard = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-lg',
    'p-6',
    'w-full',
    'max-w-md',
    'min-w-[200px]'
  );

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
    'disabled:bg-gray-100',
    'disabled:cursor-not-allowed',
    'disabled:text-gray-500',
    'disabled:border-gray-300'
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

  const classTitle = clsx('text-xl', 'font-bold', 'mb-6', 'text-left', {
    'text-gray-500': readonly,
    'text-gray-800': !readonly,
  });
  const classLabel = clsx('text-sm', 'font-medium', 'text-gray-700', 'mb-2', 'block');

  useEffect(() => {
    fetchItemList();
  }, [fetchItemList]);

  return (
    <div className={classCard}>
      <h1 className={classTitle}>{title}</h1>
      <label className={classLabel}>{!readonly ? 'Select Policy:' : 'Selected Policy:'}</label>

      <select
        className={classSelect}
        value={selectedPolicy || ''}
        onChange={(e) => dispatch(selectPolicyType(e.target.value))}
        disabled={fetching || loading || readonly}
      >
        <option value="" disabled={readonly}>
          Choose policy...
        </option>
        {policyList.map((item) => (
          <option key={item} value={item}>
            {item}
          </option>
        ))}
      </select>

      <div className="mb-4" />

      <label className={classLabel}>{!readonly ? 'Select Device:' : 'Selected Device:'}</label>
      <select
        className={classSelect}
        value={selectedDevice || ''}
        onChange={(e) => dispatch(selectPolicyDevice(e.target.value))}
        disabled={fetching || loading || readonly}
      >
        <option value="" disabled={readonly}>
          Choose device...
        </option>
        {deviceList.map((item) => (
          <option key={item} value={item}>
            {item}
          </option>
        ))}
      </select>
      {!readonly && (
        <>
          <div className="mb-4" />
          <button
            className={classRefreshButton}
            onClick={fetchItemList}
            disabled={fetching || loading || isTraining || readonly}
          >
            <div className="flex items-center justify-center gap-2">
              <MdRefresh size={16} className={fetching ? 'animate-spin' : ''} />
              {fetching ? 'Loading...' : `Refresh`}
            </div>
          </button>
        </>
      )}
    </div>
  );
}
