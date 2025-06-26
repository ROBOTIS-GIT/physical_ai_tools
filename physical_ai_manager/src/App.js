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

import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import { MdHome, MdVideocam, MdMemory } from 'react-icons/md';
import { Toaster } from 'react-hot-toast';
import toast from 'react-hot-toast';
import './App.css';
import HomePage from './pages/HomePage';
import RecordPage from './pages/RecordPage';
import InferencePage from './pages/InferencePage';
import SettingPage from './pages/SettingPage';
import { useRosTaskStatus } from './hooks/useRosTaskStatus';

function App() {
  const defaultRosHost = window.location.hostname + ':8080';
  const [page, setPage] = useState('home');
  const [topics, setTopics] = useState([null, null, null, null]);
  const [rosHost, setRosHost] = useState(defaultRosHost);
  const [currentRobotType, setCurrentRobotType] = useState('');
  const isFirstLoad = useRef(true);

  // Subscribe to task status from ROS topic (always active)
  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;
  const { taskStatus, taskInfo, updateTaskInfo, updateTaskStatus } = useRosTaskStatus(
    rosbridgeUrl,
    '/task/status'
  );

  useEffect(() => {
    if (isFirstLoad.current && page === 'home' && taskStatus.topicReceived) {
      if (taskInfo?.taskType === 'record') {
        setPage('record');
      } else if (taskInfo?.taskType === 'inference') {
        setPage('inference');
      }
      isFirstLoad.current = false;
    }
  }, [page, taskStatus, taskInfo]);

  // Load YAML content from local storage
  const [yamlContent, setYamlContent] = useState(() => {
    const savedContent = localStorage.getItem('yamlFileContent');
    try {
      return savedContent ? JSON.parse(savedContent) : null;
    } catch (error) {
      console.error('Error parsing YAML data from local storage:', error);
      return null;
    }
  });

  const handleHomePageNavigation = () => {
    isFirstLoad.current = false;
    setPage('home');
  };

  // Check conditions for Record page navigation
  const handleRecordPageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleRecordPageNavigation');
      isFirstLoad.current = false;
      setPage('record');
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log('robot type:', taskStatus.robotType, '=> allowing navigation to Record page');
      isFirstLoad.current = false;
      setPage('record');
      return;
    }

    // Block navigation if robot type is not set
    if (!currentRobotType || currentRobotType.trim() === '') {
      toast.error('Please select a robot type first in the Home page', {
        duration: 4000,
      });
      console.log('Robot type not set, blocking navigation to Record page');
      return;
    }

    // Allow navigation if conditions are met
    console.log('Robot type set, allowing navigation to Record page');
    setPage('record');
  };

  const handleInferencePageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleInferencePageNavigation');
      isFirstLoad.current = false;
      setPage('inference');
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log('robot type:', taskStatus.robotType, '=> allowing navigation to Inference page');
      isFirstLoad.current = false;
      setPage('inference');
      return;
    }

    // Block navigation if robot type is not set
    if (!currentRobotType || currentRobotType.trim() === '') {
      toast.error('Please select a robot type first in the Home page', {
        duration: 4000,
      });
      console.log('Robot type not set, blocking navigation to Inference page');
      return;
    }

    // Allow navigation if conditions are met
    console.log('Robot type set, allowing navigation to Inference page');
    setPage('inference');
  };

  return (
    <div className="flex h-screen w-screen">
      <aside className="w-30 bg-gray-100 h-full flex flex-col items-center pt-10 gap-4 shadow-[inset_0_0_2px_rgba(0,0,0,0.1)]">
        <button
          className={clsx(
            'flex',
            'flex-col',
            'items-center',
            'rounded-2xl',
            'border-none',
            'py-5',
            'px-4',
            'text-base',
            'text-gray-800',
            'cursor-pointer',
            'transition-colors',
            'duration-150',
            'outline-none',
            'min-w-24',
            {
              'hover:bg-gray-200 active:bg-gray-400': page !== 'home',
              'bg-gray-300': page === 'home',
            }
          )}
          onClick={handleHomePageNavigation}
        >
          <MdHome size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Home</span>
        </button>
        <button
          className={clsx(
            'flex',
            'flex-col',
            'items-center',
            'rounded-2xl',
            'border-none',
            'py-5',
            'px-4',
            'text-base',
            'text-gray-800',
            'cursor-pointer',
            'transition-colors',
            'duration-150',
            'outline-none',
            'min-w-24',
            {
              'hover:bg-gray-200 active:bg-gray-400': page !== 'record',
              'bg-gray-300': page === 'record',
            }
          )}
          onClick={handleRecordPageNavigation}
        >
          <MdVideocam size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Record</span>
        </button>
        <button
          className={clsx(
            'flex',
            'flex-col',
            'items-center',
            'rounded-2xl',
            'border-none',
            'py-5',
            'px-4',
            'text-base',
            'text-gray-800',
            'cursor-pointer',
            'transition-colors',
            'duration-150',
            'outline-none',
            'w-24',
            {
              'hover:bg-gray-200 active:bg-gray-400': page !== 'inference',
              'bg-gray-300': page === 'inference',
            }
          )}
          onClick={handleInferencePageNavigation}
        >
          <MdMemory size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Inference</span>
        </button>
      </aside>
      <main className="flex-1 flex flex-col h-screen min-h-0">
        {page === 'home' ? (
          <HomePage
            topics={topics}
            setTopics={setTopics}
            rosHost={rosHost}
            currentRobotType={currentRobotType}
            setCurrentRobotType={setCurrentRobotType}
            taskStatus={taskStatus}
            updateTaskStatus={updateTaskStatus}
          />
        ) : page === 'record' ? (
          <RecordPage
            topics={topics}
            setTopics={setTopics}
            rosHost={rosHost}
            yamlContent={yamlContent}
            taskStatus={taskStatus}
            taskInfo={taskInfo}
            updateTaskInfo={updateTaskInfo}
            isActive={page === 'record'}
          />
        ) : page === 'inference' ? (
          <InferencePage
            topics={topics}
            setTopics={setTopics}
            rosHost={rosHost}
            taskStatus={taskStatus}
            taskInfo={taskInfo}
            isActive={page === 'inference'}
          />
        ) : (
          <SettingPage
            rosHost={rosHost}
            setRosHost={setRosHost}
            yamlContent={yamlContent}
            setYamlContent={setYamlContent}
          />
        )}
      </main>
      <Toaster
        position="top-center"
        gutter={8}
        toastOptions={{
          duration: 3000,
          style: {
            background: '#363636',
            color: '#fff',
          },
          success: {
            duration: 3000,
            style: {
              background: '#10b981',
            },
          },
          error: {
            duration: 6000,
            style: {
              background: '#ef4444',
            },
          },
        }}
      />
    </div>
  );
}

export default App;
