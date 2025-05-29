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

import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import ImageGrid from './components/ImageGrid';
import EpisodeStatus from './components/EpisodeStatus';
import ControlPanel from './components/ControlPanel';
import InfoPanel from './components/InfoPanel';
import { MdHome } from 'react-icons/md';
import RosTopicList from './components/RosTopicList';
import RosTopicSubscriber from './components/RosTopicSubscriber';
import RosServiceCaller from './components/RosServiceCaller';
import './App.css';
import YamlEditor from './components/YamlEditor';
import { useRosServiceCaller } from './hooks/useRosServiceCaller';

function SettingPage({ rosHost, setRosHost, yamlContent, setYamlContent }) {
  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;

  return (
    <div className="w-full h-full flex items-start justify-center text-xl flex-row gap-8">
      <div className="flex flex-col items-center mt-10 w-full">
        <div className="mb-6 text-3xl font-semibold">
          Image Streaming Server Address Configuration
        </div>
        <div className="text-gray-500 text-base">
          Current auto-set value: <b>{rosHost}</b>
          <br />
          (This value is automatically determined based on window.location.hostname)
        </div>
        <div className="mt-10 w-3/5">
          <YamlEditor onYamlLoad={setYamlContent} />
        </div>
      </div>
      <div className="flex flex-col items-stretch">
        <RosTopicList rosbridgeUrl={rosbridgeUrl} />
        <RosTopicSubscriber
          rosbridgeUrl={rosbridgeUrl}
          topicName="/chatter"
          messageType="std_msgs/String"
        />
        <RosServiceCaller
          rosbridgeUrl={rosbridgeUrl}
          serviceName="/gui/set_page"
          serviceType="gaemi_interfaces/srv/SetGUIPage"
          requestFields={{ page_name: '', option: '' }}
        />
      </div>
    </div>
  );
}

function HomePage({ topics, setTopics, rosHost, yamlContent }) {
  const [info, setInfo] = useState({
    taskName: 'ai_worker_task_abcd_12345',
    robotType: 'ai-worker',
    taskType: 'record',
    taskInstruction: 'pick and place objects',
    repoId: 'robotis/ai_worker_dataset',
    fps: 30,
    tags: ['tutorial'],
    warmupTime: 5,
    episodeTime: 60,
    resetTime: 60,
    numEpisodes: 100,
    resume: true,
    pushToHub: true,
  });
  const [isRightPanelCollapsed, setIsRightPanelCollapsed] = useState(false);

  const { setGuiPage, sendRecordCommand } = useRosServiceCaller(
    `ws://${rosHost.split(':')[0]}:9090`
  );

  const handleControlCommand = (cmd) => {
    console.log('Control command received:', cmd);
    try {
      if (cmd === 'Start') {
        console.log('Calling sendRecordCommand with start');
        sendRecordCommand('start', info);
      } else if (cmd === 'Stop') {
        console.log('Calling sendRecordCommand with stop');
        sendRecordCommand('stop', info);
      } else if (cmd === 'Retry') {
        console.log('Calling sendRecordCommand with retry');
        sendRecordCommand('retry', info);
      } else if (cmd === 'Next') {
        console.log('Calling sendRecordCommand with next');
        sendRecordCommand('next', info);
      } else if (cmd === 'Finish') {
        console.log('Calling sendRecordCommand with finish');
        sendRecordCommand('finish', info);
      }
    } catch (error) {
      console.error('Error handling control command:', error);
    }
  };

  return (
    <div className="h-full flex flex-col overflow-hidden">
      {/* Main content area */}
      <div className="flex-1 flex min-h-0 pt-0 px-0 justify-center items-start overflow-hidden">
        <div
          className={clsx(
            'transition-all',
            'duration-300',
            'ease-in-out',
            'flex',
            'items-center',
            'justify-center',
            'min-h-0',
            'h-full',
            'overflow-hidden',
            'm-2',
            {
              'flex-[12]': isRightPanelCollapsed,
              'flex-[10]': !isRightPanelCollapsed,
            }
          )}
        >
          <ImageGrid topics={topics} setTopics={setTopics} rosHost={rosHost} />
        </div>
        <div
          className={clsx('transition-all', 'duration-300', 'ease-in-out', 'relative', {
            'flex-[0_0_40px]': isRightPanelCollapsed,
            'flex-[1]': !isRightPanelCollapsed,
            'min-w-[60px]': isRightPanelCollapsed,
            'min-w-[400px]': !isRightPanelCollapsed,
            'max-w-[60px]': isRightPanelCollapsed,
            'max-w-[400px]': !isRightPanelCollapsed,
          })}
        >
          <button
            onClick={() => setIsRightPanelCollapsed(!isRightPanelCollapsed)}
            className={clsx(
              'absolute',
              'top-10',
              'bg-white',
              'border',
              'border-gray-300',
              'rounded-full',
              'w-12',
              'h-12',
              'flex',
              'items-center',
              'justify-center',
              'shadow-md',
              'hover:bg-gray-50',
              'transition-all',
              'duration-200',
              'z-10',
              {
                'left-2': isRightPanelCollapsed,
                'left-[10px]': !isRightPanelCollapsed,
              }
            )}
          >
            <span
              className={clsx(
                'text-gray-600',
                'text-base',
                'font-bold',
                'transition-transform',
                'duration-200',
                {
                  'rotate-180': !isRightPanelCollapsed,
                }
              )}
            >
              ◀
            </span>
          </button>
          <div
            className={clsx('h-full', 'overflow-hidden', 'transition-opacity', 'duration-300', {
              'opacity-0': isRightPanelCollapsed,
              'opacity-100': !isRightPanelCollapsed,
              'pointer-events-none': isRightPanelCollapsed,
              'pointer-events-auto': !isRightPanelCollapsed,
            })}
          >
            <div className="ml-1 mr-1 flex flex-col items-center h-full">
              <div className="w-full h-10"></div>
              <div className="w-[250px] flex justify-center">
                <EpisodeStatus />
              </div>
              <div className="w-full h-10"></div>
              <div className="flex-1 overflow-y-auto">
                <InfoPanel info={info} onChange={setInfo} />
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Control Panel - Fixed at bottom */}
      <div className="flex-shrink-0">
        <ControlPanel onCommand={handleControlCommand} />
      </div>
    </div>
  );
}

function App() {
  const defaultRosHost = window.location.hostname + ':8080';
  const [page, setPage] = useState('home');
  const [topics, setTopics] = useState([null, null, null, null]);
  const [rosHost, setRosHost] = useState(defaultRosHost);

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

  // Debug log for yamlContent
  useEffect(() => {
    console.log('App.js - yamlContent state:', yamlContent);
  }, [yamlContent]);

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
              'hover:bg-gray-300 active:bg-gray-400': page !== 'home',
              'bg-gray-300': page === 'home',
            }
          )}
          onClick={() => setPage('home')}
        >
          <MdHome size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Home</span>
        </button>
      </aside>
      <main className="flex-1 flex flex-col h-screen min-h-0">
        {page === 'home' ? (
          <HomePage
            topics={topics}
            setTopics={setTopics}
            rosHost={rosHost}
            yamlContent={yamlContent}
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
    </div>
  );
}

export default App;
