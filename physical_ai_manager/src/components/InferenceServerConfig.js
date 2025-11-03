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
// Author: Dongyun Kim

import React, { useState, useCallback, useEffect } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdFolderOpen, MdSettings, MdRefresh } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import FileBrowserModal from './FileBrowserModal';
import { DEFAULT_PATHS } from '../constants/paths';

const InferenceServerConfig = ({ disabled = false }) => {
  const taskStatus = useSelector((state) => state.tasks.taskStatus);

  // Server configuration states
  const [serverIp, setServerIp] = useState('0.0.0.0');
  const [serverPort, setServerPort] = useState(5555);
  const [policyType, setPolicyType] = useState('GR00T_N1_5_TRT');
  const [policyPath, setPolicyPath] = useState('');
  const [robotType, setRobotType] = useState('');

  // UI states
  const [showPolicyPathModal, setShowPolicyPathModal] = useState(false);
  const [isConfiguring, setIsConfiguring] = useState(false);
  const [isExpanded, setIsExpanded] = useState(true);

  const { setInferenceServerInfo } = useRosServiceCaller();

  // Update robot type from task status
  useEffect(() => {
    if (taskStatus?.robotType) {
      setRobotType(taskStatus.robotType);
    }
  }, [taskStatus?.robotType]);

  const handlePolicyPathSelect = useCallback((item) => {
    setPolicyPath(item.full_path);
    setShowPolicyPathModal(false);
  }, []);

  const handleConfigure = useCallback(async () => {
    // Validation
    if (!serverIp.trim()) {
      toast.error('Please enter server IP address');
      return;
    }

    if (!serverPort || serverPort < 1 || serverPort > 65535) {
      toast.error('Please enter a valid port number (1-65535)');
      return;
    }

    if (!policyType.trim()) {
      toast.error('Please select a policy type');
      return;
    }

    if (!policyPath.trim()) {
      toast.error('Please select a policy path');
      return;
    }

    if (!robotType.trim()) {
      toast.error('Robot type is not set. Please set robot type first.');
      return;
    }

    setIsConfiguring(true);
    try {
      const result = await setInferenceServerInfo({
        server_ip: serverIp,
        server_port: parseInt(serverPort, 10),
        policy_type: policyType,
        policy_path: policyPath,
        robot_type: robotType,
      });

      if (result && result.success) {
        toast.success('Inference server configured successfully!');
      } else {
        toast.error(result?.message || 'Failed to configure inference server');
      }
    } catch (error) {
      console.error('Error configuring inference server:', error);
      toast.error(`Failed to configure server: ${error.message}`);
    } finally {
      setIsConfiguring(false);
    }
  }, [serverIp, serverPort, policyType, policyPath, robotType, setInferenceServerInfo]);

  const handleReset = useCallback(() => {
    setServerIp('0.0.0.0');
    setServerPort(5555);
    setPolicyType('GR00T_N1_5_TRT');
    setPolicyPath('');
    toast.success('Configuration reset to defaults');
  }, []);

  const classContainer = clsx(
    'bg-white',
    'border',
    'border-gray-200',
    'rounded-2xl',
    'shadow-md',
    'p-4',
    'w-full',
    'max-w-[350px]',
    'transition-all',
    'duration-200',
    {
      'opacity-50': disabled,
      'pointer-events-none': disabled,
    }
  );

  const classHeader = clsx(
    'flex',
    'items-center',
    'justify-between',
    'mb-4',
    'cursor-pointer',
    'select-none'
  );

  const classTitle = clsx('text-lg', 'font-semibold', 'text-gray-800', 'flex', 'items-center');

  const classFormGroup = clsx('mb-3', 'last:mb-0');

  const classLabel = clsx('text-sm', 'text-gray-600', 'mb-1', 'font-medium', 'block');

  const classInput = clsx(
    'text-sm',
    'w-full',
    'px-3',
    'py-2',
    'border',
    'border-gray-300',
    'rounded-lg',
    'focus:outline-none',
    'focus:ring-2',
    'focus:ring-blue-500',
    'focus:border-transparent',
    'transition-all',
    'duration-200',
    {
      'bg-gray-50': disabled,
      'cursor-not-allowed': disabled,
    }
  );

  const classSelect = clsx(classInput);

  const classFilePathContainer = clsx('flex', 'items-center', 'gap-2');

  const classFilePathInput = clsx(classInput, 'flex-1');

  const classBrowseButton = clsx(
    'px-3',
    'py-2',
    'bg-gray-100',
    'border',
    'border-gray-300',
    'rounded-lg',
    'hover:bg-gray-200',
    'transition-colors',
    'duration-200',
    'flex',
    'items-center',
    'justify-center',
    {
      'opacity-50': disabled,
      'cursor-not-allowed': disabled,
    }
  );

  const classButtonGroup = clsx('flex', 'gap-2', 'mt-4');

  const classButton = clsx(
    'flex-1',
    'px-4',
    'py-2',
    'rounded-lg',
    'font-medium',
    'transition-all',
    'duration-200',
    'flex',
    'items-center',
    'justify-center',
    'gap-2'
  );

  const classConfigureButton = clsx(
    classButton,
    'bg-blue-500',
    'text-white',
    'hover:bg-blue-600',
    'disabled:bg-gray-300',
    'disabled:cursor-not-allowed'
  );

  const classResetButton = clsx(
    classButton,
    'bg-gray-100',
    'text-gray-700',
    'hover:bg-gray-200',
    'disabled:bg-gray-100',
    'disabled:cursor-not-allowed'
  );

  const classCollapsibleContent = clsx('transition-all', 'duration-300', 'overflow-hidden', {
    'max-h-0': !isExpanded,
    'max-h-[1000px]': isExpanded,
  });

  const policyTypes = [
    { value: 'GR00T_N1_5_TRT', label: 'GR00T N1.5 (TensorRT)' },
    { value: 'GR00T_N1_5', label: 'GR00T N1.5 (PyTorch)' },
    { value: 'LEROBOT_ACT', label: 'LeRobot ACT' },
    { value: 'LEROBOT_DIFFUSION', label: 'LeRobot Diffusion' },
  ];

  return (
    <div className={classContainer}>
      <div className={classHeader} onClick={() => setIsExpanded(!isExpanded)}>
        <div className={classTitle}>
          <MdSettings className="mr-2 text-blue-500" size={24} />
          Inference Server Config
        </div>
        <span className="text-gray-400">
          {isExpanded ? '▼' : '▶'}
        </span>
      </div>

      <div className={classCollapsibleContent}>
        {/* Server IP */}
        <div className={classFormGroup}>
          <label className={classLabel}>Server IP Address</label>
          <input
            type="text"
            className={classInput}
            value={serverIp}
            onChange={(e) => setServerIp(e.target.value)}
            placeholder="e.g., 192.168.1.100"
            disabled={disabled}
          />
        </div>

        {/* Server Port */}
        <div className={classFormGroup}>
          <label className={classLabel}>Server Port</label>
          <input
            type="number"
            className={classInput}
            value={serverPort}
            onChange={(e) => setServerPort(e.target.value)}
            placeholder="e.g., 5555"
            min="1"
            max="65535"
            disabled={disabled}
          />
        </div>

        {/* Policy Type */}
        <div className={classFormGroup}>
          <label className={classLabel}>Policy Type</label>
          <select
            className={classSelect}
            value={policyType}
            onChange={(e) => setPolicyType(e.target.value)}
            disabled={disabled}
          >
            {policyTypes.map((type) => (
              <option key={type.value} value={type.value}>
                {type.label}
              </option>
            ))}
          </select>
        </div>

        {/* Policy Path */}
        <div className={classFormGroup}>
          <label className={classLabel}>Policy Model Path</label>
          <div className={classFilePathContainer}>
            <input
              type="text"
              className={classFilePathInput}
              value={policyPath}
              onChange={(e) => setPolicyPath(e.target.value)}
              placeholder="Select policy model path..."
              disabled={disabled}
            />
            <button
              className={classBrowseButton}
              onClick={() => setShowPolicyPathModal(true)}
              disabled={disabled}
              title="Browse"
            >
              <MdFolderOpen size={20} />
            </button>
          </div>
        </div>

        {/* Robot Type (Read-only) */}
        <div className={classFormGroup}>
          <label className={classLabel}>Robot Type</label>
          <input
            type="text"
            className={clsx(classInput, 'bg-gray-100', 'cursor-not-allowed')}
            value={robotType}
            readOnly
            disabled
            placeholder="Set robot type first..."
          />
        </div>

        {/* Action Buttons */}
        <div className={classButtonGroup}>
          <button
            className={classConfigureButton}
            onClick={handleConfigure}
            disabled={disabled || isConfiguring}
          >
            {isConfiguring ? (
              <>
                <span className="animate-spin">⏳</span>
                Configuring...
              </>
            ) : (
              <>
                <MdSettings size={18} />
                Configure Server
              </>
            )}
          </button>
          <button
            className={classResetButton}
            onClick={handleReset}
            disabled={disabled || isConfiguring}
          >
            <MdRefresh size={18} />
            Reset
          </button>
        </div>
      </div>

      {/* File Browser Modal */}
      {showPolicyPathModal && (
        <FileBrowserModal
          isOpen={showPolicyPathModal}
          onClose={() => setShowPolicyPathModal(false)}
          onSelect={handlePolicyPathSelect}
          initialPath={DEFAULT_PATHS.POLICY_MODEL_PATH}
          targetFiles={['*']}
          title="Select Policy Model"
          selectMode="folder"
        />
      )}
    </div>
  );
};

export default InferenceServerConfig;
