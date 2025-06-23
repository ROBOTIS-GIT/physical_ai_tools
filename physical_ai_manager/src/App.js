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

import React, { useEffect, useRef } from 'react';
import clsx from 'clsx';
import { MdHome, MdVideocam } from 'react-icons/md';
import { Toaster } from 'react-hot-toast';
import toast from 'react-hot-toast';
import './App.css';
import HomePage from './pages/HomePage';
import RecordPage from './pages/RecordPage';
import SettingPage from './pages/SettingPage';

// Flux imports
import { useAppStore } from './flux/hooks/useAppStore';
import { useTaskStore } from './flux/hooks/useTaskStore';
import AppActions from './flux/actions/AppActions';

import { useRosTaskStatus } from './hooks/useRosTaskStatus';

function App() {
  const isFirstLoad = useRef(true);

  // Get state from Flux stores
  const { currentPage, currentRobotType, error, rosHost } = useAppStore();

  const { taskStatus } = useTaskStore();

  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;

  const { rosTaskStatus } = useRosTaskStatus(rosbridgeUrl, '/task/status');

  // Handle automatic navigation to record page on first load
  useEffect(() => {
    if (isFirstLoad.current && currentPage === 'home' && taskStatus.topicReceived) {
      AppActions.navigateToPage('record');
      AppActions.setFirstLoad(false);
      isFirstLoad.current = false;
    }
  }, [currentPage, taskStatus]);

  // Handle errors with toast notifications
  useEffect(() => {
    if (error) {
      toast.error(error);
      AppActions.clearError();
    }
  }, [error]);

  const handleHomePageNavigation = () => {
    isFirstLoad.current = false;
    AppActions.setFirstLoad(false);
    AppActions.navigateToPage('home');
  };

  // Check conditions for Record page navigation
  const handleRecordPageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleRecordPageNavigation');
      isFirstLoad.current = false;
      AppActions.setFirstLoad(false);
      AppActions.navigateToPage('record');
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log('robot type:', taskStatus.robotType, '=> allowing navigation to Record page');
      isFirstLoad.current = false;
      AppActions.setFirstLoad(false);
      AppActions.navigateToPage('record');
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
    AppActions.navigateToPage('record');
  };

  return (
    <div className="flex h-screen w-screen">
      <aside className="w-30 bg-gray-200 h-full flex flex-col items-center pt-10 gap-6">
        <button
          className={clsx(
            'flex',
            'flex-col',
            'items-center',
            'bg-gray-100',
            'rounded-2xl',
            'border-none',
            'py-5',
            'px-4',
            'mb-3',
            'text-base',
            'text-gray-800',
            'cursor-pointer',
            'transition-colors',
            'duration-150',
            'outline-none',
            'min-w-20',
            {
              'hover:bg-gray-300 active:bg-gray-400': currentPage !== 'home',
              'bg-gray-300': currentPage === 'home',
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
            'bg-gray-100',
            'rounded-2xl',
            'border-none',
            'py-5',
            'px-4',
            'mb-3',
            'text-base',
            'text-gray-800',
            'cursor-pointer',
            'transition-colors',
            'duration-150',
            'outline-none',
            'min-w-20',
            {
              'hover:bg-gray-300 active:bg-gray-400': currentPage !== 'record',
              'bg-gray-300': currentPage === 'record',
            }
          )}
          onClick={handleRecordPageNavigation}
        >
          <MdVideocam size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Record</span>
        </button>
      </aside>
      <main className="flex-1 flex flex-col h-screen min-h-0">
        {currentPage === 'home' ? (
          <HomePage />
        ) : currentPage === 'record' ? (
          <RecordPage />
        ) : (
          <SettingPage />
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
            duration: 4000,
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
