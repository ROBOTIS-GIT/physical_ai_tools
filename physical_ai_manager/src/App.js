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
import { MdHome, MdVideocam, MdMemory } from 'react-icons/md';
import { Toaster } from 'react-hot-toast';
import toast from 'react-hot-toast';
import './App.css';
import HomePage from './pages/HomePage';
import RecordPage from './pages/RecordPage';
import InferencePage from './pages/InferencePage';
import { useRosTaskStatus } from './hooks/useRosTaskStatus';
import rosConnectionManager from './utils/rosConnectionManager';
import { useDispatch, useSelector } from 'react-redux';
import { setRosHost } from './features/ros/rosSlice';
import { moveToPage } from './features/ui/uiSlice';
import PageType from './constants/pageType';

function App() {
  const dispatch = useDispatch();
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const taskInfo = useSelector((state) => state.tasks.taskInfo);

  const defaultRosHost = window.location.hostname;
  dispatch(setRosHost(defaultRosHost));

  const page = useSelector((state) => state.ui.currentPage);
  const robotType = useSelector((state) => state.tasks.taskStatus.robotType);

  const isFirstLoad = useRef(true);

  // Subscribe to task status from ROS topic (always active)
  useRosTaskStatus();

  // Disconnect ROS connection when app unmounts
  useEffect(() => {
    return () => {
      console.log('App unmounting, cleaning up global ROS connection');
      rosConnectionManager.disconnect();
    };
  }, []);

  useEffect(() => {
    if (isFirstLoad.current && page === PageType.HOME && taskStatus.topicReceived) {
      if (taskInfo?.taskType === PageType.RECORD) {
        dispatch(moveToPage(PageType.RECORD));
      } else if (taskInfo?.taskType === PageType.INFERENCE) {
        dispatch(moveToPage(PageType.INFERENCE));
      }
      isFirstLoad.current = false;
    }
  }, [page, taskInfo?.taskType, taskStatus.topicReceived, dispatch]);

  const handleHomePageNavigation = () => {
    isFirstLoad.current = false;
    dispatch(moveToPage(PageType.HOME));
  };

  // Check conditions for Record page navigation
  const handleRecordPageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleRecordPageNavigation');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.RECORD));
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log('robot type:', taskStatus.robotType, '=> allowing navigation to Record page');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.RECORD));
      return;
    }

    // Block navigation if robot type is not set
    if (!robotType || robotType.trim() === '') {
      toast.error('Please select a robot type first in the Home page', {
        duration: 4000,
      });
      console.log('Robot type not set, blocking navigation to Record page');
      return;
    }

    // Allow navigation if conditions are met
    console.log('Robot type set, allowing navigation to Record page');
    dispatch(moveToPage(PageType.RECORD));
  };

  const handleInferencePageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleInferencePageNavigation');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.INFERENCE));
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log('robot type:', taskStatus.robotType, '=> allowing navigation to Inference page');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.INFERENCE));
      return;
    }

    // Block navigation if robot type is not set
    if (!robotType || robotType.trim() === '') {
      toast.error('Please select a robot type first in the Home page', {
        duration: 4000,
      });
      console.log('Robot type not set, blocking navigation to Inference page');
      return;
    }

    // Allow navigation if conditions are met
    console.log('Robot type set, allowing navigation to Inference page');
    dispatch(moveToPage(PageType.INFERENCE));
  };

  // Force cleanup of all image streams when page changes
  useEffect(() => {
    return () => {
      // Clean up all possible image streams when page changes
      console.log('Page changing, forcing complete cleanup of all image streams');

      // Find all streaming images by src pattern
      const allStreamImgs = document.querySelectorAll('img[src*="/stream"]');
      allStreamImgs.forEach((img, index) => {
        img.src = '';
        if (img.parentNode) {
          img.parentNode.removeChild(img);
        }
        console.log(`Page cleanup: removed stream image ${index}`);
      });

      console.log(`Page cleanup completed: removed ${allStreamImgs.length} streaming images`);
    };
  }, [page]);

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
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.HOME,
              'bg-gray-300': page === PageType.HOME,
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
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.RECORD,
              'bg-gray-300': page === PageType.RECORD,
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
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.INFERENCE,
              'bg-gray-300': page === PageType.INFERENCE,
            }
          )}
          onClick={handleInferencePageNavigation}
        >
          <MdMemory size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Inference</span>
        </button>
      </aside>
      <main className="flex-1 flex flex-col h-screen min-h-0">
        {page === PageType.HOME ? (
          <HomePage />
        ) : page === PageType.RECORD ? (
          <RecordPage isActive={page === PageType.RECORD} />
        ) : page === PageType.INFERENCE ? (
          <InferencePage isActive={page === PageType.INFERENCE} />
        ) : (
          <HomePage />
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
