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
import { MdHome, MdVideocam, MdMemory, MdWidgets } from 'react-icons/md';
import { GoGraph } from 'react-icons/go';
import { Toaster } from 'react-hot-toast';
import toast from 'react-hot-toast';
import './App.css';
import HomePage from './pages/HomePage';
import RecordPage from './pages/RecordPage';
import InferencePage from './pages/InferencePage';
import TrainingPage from './pages/TrainingPage';
import EditDatasetPage from './pages/EditDatasetPage';
import { useRosTopicSubscription } from './hooks/useRosTopicSubscription';
import rosConnectionManager from './utils/rosConnectionManager';
import { useDispatch, useSelector } from 'react-redux';
import { setRosHost } from './features/ros/rosSlice';
import { moveToPage } from './features/ui/uiSlice';
import PageType from './constants/pageType';

function App() {
  const dispatch = useDispatch();
  const taskStatus = useSelector((state) => state.tasks.taskStatus);
  const taskInfo = useSelector((state) => state.tasks.taskInfo);
  const trainingTopicReceived = useSelector((state) => state.training.topicReceived);

  const defaultRosHost = window.location.hostname;
  dispatch(setRosHost(defaultRosHost));

  const page = useSelector((state) => state.ui.currentPage);
  const robotType = useSelector((state) => state.tasks.taskStatus.robotType);

  const isFirstLoad = useRef(true);

  // Subscribe to task status from ROS topic (always active)
  const rosSubscriptionControls = useRosTopicSubscription();
  rosConnectionManager.setOnConnected(rosSubscriptionControls.initializeSubscriptions);

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
    } else if (isFirstLoad.current && page === PageType.HOME && trainingTopicReceived) {
      dispatch(moveToPage(PageType.TRAINING));
      isFirstLoad.current = false;
    }
  }, [page, taskInfo?.taskType, taskStatus.topicReceived, trainingTopicReceived, dispatch]);

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

  const handleTrainingPageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleTrainingPageNavigation');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.TRAINING));
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log('robot type:', taskStatus.robotType, '=> allowing navigation to Training page');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.TRAINING));
      return;
    }

    // Block navigation if robot type is not set
    if (!robotType || robotType.trim() === '') {
      toast.error('Please select a robot type first in the Home page', {
        duration: 4000,
      });
      console.log('Robot type not set, blocking navigation to Training page');
      return;
    }

    // Allow navigation if conditions are met
    console.log('Robot type set, allowing navigation to Training page');
    dispatch(moveToPage(PageType.TRAINING));
  };

  const handleEditDatasetPageNavigation = () => {
    if (process.env.REACT_APP_DEBUG === 'true') {
      console.log('handleEditDatasetPageNavigation');
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.EDIT_DATASET));
      return;
    }

    // Allow navigation if task is in progress
    if (taskStatus && taskStatus.robotType !== '') {
      console.log(
        'robot type:',
        taskStatus.robotType,
        '=> allowing navigation to Edit Dataset page'
      );
      isFirstLoad.current = false;
      dispatch(moveToPage(PageType.EDIT_DATASET));
      return;
    }

    // Block navigation if robot type is not set
    if (!robotType || robotType.trim() === '') {
      toast.error('Please select a robot type first in the Home page', {
        duration: 4000,
      });
      return;
    }

    // Allow navigation if conditions are met
    dispatch(moveToPage(PageType.EDIT_DATASET));
  };

  // Force cleanup of all image streams when page changes
  useEffect(() => {
    return () => {
      // Clean up all possible image streams when page changes

      // Find all streaming images by src pattern
      const allStreamImgs = document.querySelectorAll('img[src*="/stream"]');
      allStreamImgs.forEach((img, index) => {
        img.src = '';
        if (img.parentNode) {
          img.parentNode.removeChild(img);
        }
      });
    };
  }, [page]);

  const classPageButton = clsx(
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
    'w-24'
  );

  return (
    <div className="flex min-h-screen w-screen">
      <aside className="w-30 min-w-28 bg-gray-100 min-h-screen flex flex-col items-center gap-4 shadow-[inset_0_0_2px_rgba(0,0,0,0.1)]">
        <div className="w-full h-screen flex flex-col gap-2 items-center overflow-y-auto scrollbar-thin">
          <div className="w-full h-8"></div>
          {/* Home page button */}
          <button
            className={clsx(classPageButton, {
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.HOME,
              'bg-gray-300': page === PageType.HOME,
            })}
            onClick={handleHomePageNavigation}
          >
            <MdHome size={32} className="mb-1.5" />
            <span className="mt-1 text-sm">Home</span>
          </button>

          {/* Record page button */}
          <button
            className={clsx(classPageButton, {
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.RECORD,
              'bg-gray-300': page === PageType.RECORD,
            })}
            onClick={handleRecordPageNavigation}
          >
            <MdVideocam size={32} className="mb-1.5" />
            <span className="mt-1 text-sm">Record</span>
          </button>
          {/* Training page button */}
          <button
            className={clsx(classPageButton, {
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.TRAINING,
              'bg-gray-300': page === PageType.TRAINING,
            })}
            onClick={handleTrainingPageNavigation}
          >
            <GoGraph size={28} className="mb-1.5" />
            <span className="mt-1 text-sm">Training</span>
          </button>
          {/* Inference page button */}
          <button
            className={clsx(classPageButton, {
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.INFERENCE,
              'bg-gray-300': page === PageType.INFERENCE,
            })}
            onClick={handleInferencePageNavigation}
          >
            <MdMemory size={32} className="mb-1.5" />
            <span className="mt-1 text-sm">Inference</span>
          </button>

          {/* Divider line */}
          <div className="w-24 h-1 border-t-2 rounded-full border-gray-200 mt-3"></div>

          {/* Edit dataset page button */}
          <button
            className={clsx(classPageButton, {
              'hover:bg-gray-200 active:bg-gray-400': page !== PageType.EDIT_DATASET,
              'bg-gray-300': page === PageType.EDIT_DATASET,
            })}
            onClick={handleEditDatasetPageNavigation}
          >
            <MdWidgets size={28} className="mb-2" />
            <span className="mt-1 text-sm whitespace-nowrap">Data Tools</span>
          </button>
        </div>
      </aside>
      <main className="flex-1 flex flex-col h-screen">
        {page === PageType.HOME ? (
          <HomePage />
        ) : page === PageType.RECORD ? (
          <RecordPage isActive={page === PageType.RECORD} />
        ) : page === PageType.INFERENCE ? (
          <InferencePage isActive={page === PageType.INFERENCE} />
        ) : page === PageType.TRAINING ? (
          <TrainingPage isActive={page === PageType.TRAINING} />
        ) : page === PageType.EDIT_DATASET ? (
          <EditDatasetPage isActive={page === PageType.EDIT_DATASET} />
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
            maxWidth: '500px',
            wordWrap: 'break-word',
            whiteSpace: 'pre-wrap',
            lineHeight: '1.4',
          },
          success: {
            duration: 3000,
            style: {
              background: '#10b981',
              maxWidth: '500px',
              wordWrap: 'break-word',
              whiteSpace: 'pre-wrap',
              lineHeight: '1.4',
            },
          },
          error: {
            duration: 6000,
            style: {
              background: '#ef4444',
              maxWidth: '500px',
              wordWrap: 'break-word',
              whiteSpace: 'pre-wrap',
              lineHeight: '1.4',
            },
          },
        }}
      />
    </div>
  );
}

export default App;
