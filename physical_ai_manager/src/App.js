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
import ImageGrid from './components/ImageGrid';
import TaskSelector from './components/TaskSelector';
import EpisodeStatus from './components/EpisodeStatus';
import ControlPanel from './components/ControlPanel';
import InfoPanel from './components/InfoPanel';
import { MdHome, MdSettings } from 'react-icons/md';
import RosTopicList from './components/RosTopicList';
import RosTopicSubscriber from './components/RosTopicSubscriber';
import RosServiceCaller from './components/RosServiceCaller';
import './App.css';
import YamlEditor from './components/YamlEditor';

function SettingPage({ rosHost, setRosHost, yamlContent, setYamlContent }) {
  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;

  return (
    <div
      style={{
        width: '100%',
        height: '100%',
        display: 'flex',
        alignItems: 'flex-start',
        justifyContent: 'center',
        fontSize: 22,
        flexDirection: 'row',
        gap: 32,
      }}
    >
      <div
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          marginTop: 40,
          width: '100%',
        }}
      >
        <div style={{ marginBottom: 24, fontSize: 28, fontWeight: 600 }}>
          Image Streaming Server Address Configuration
        </div>
        <div style={{ color: '#888', fontSize: 16 }}>
          Current auto-set value: <b>{rosHost}</b>
          <br />
          (This value is automatically determined based on window.location.hostname)
        </div>
        <div style={{ marginTop: 40, width: '70%' }}>
          <YamlEditor onYamlLoad={setYamlContent} />
        </div>
      </div>
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'stretch' }}>
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
  const info = {
    taskName: 'omy_task_123',
    robotType: 'omy',
    taskType: 'record',
    singleTask: 'pick and place objects',
    fps: 30,
    tags: ['tutorial'],
    warmupTime: '5 sec',
    episodeTime: '60 sec',
    resetTime: '60 sec',
    numEpisodes: 100,
    resume: true,
    pushToHub: true,
  };

  return (
    <>
      <div className="top-section">
        <div className="grid-wrapper" style={{ flex: 6, overflow: 'hidden' }}>
          <ImageGrid topics={topics} setTopics={setTopics} rosHost={rosHost} />
        </div>
        {/* <div style={{ display: 'flex', flex: 2 }}>
          <div className="right-panel">
            <div
              style={{
                width: '100%',
                height: '100px',
                minHeight: 150,
                display: 'flex',
                justifyContent: 'center',
              }}
            >
              <EpisodeStatus />
            </div>
            <div style={{ width: '100%', height: '50px' }}></div>
            <TaskSelector
              yamlContent={yamlContent}
              onTaskSelect={(task) => console.log('Selected task:', task)}
            />
            <InfoPanel info={info} />
          </div>
        </div> */}
      </div>

      {/* <ControlPanel /> */}
    </>
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
    <div className="container">
      <aside
        className="sidebar"
        style={{
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          paddingTop: 40,
          gap: 24,
        }}
      >
        <button
          className={`nav-btn${page === 'home' ? ' active' : ''}`}
          onClick={() => setPage('home')}
        >
          <MdHome size={32} style={{ marginBottom: 6 }} />
          <span>Home</span>
        </button>
        {/* <button
          className={`nav-btn${page === 'setting' ? ' active' : ''}`}
          onClick={() => setPage('setting')}
        >
          <MdSettings size={32} style={{ marginBottom: 6 }} />
          <span>Setting</span>
        </button> */}
      </aside>
      <main className="main-content">
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
