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
import ROSLIB from 'roslib';

export default function RosTopicList({ rosbridgeUrl }) {
  const [topics, setTopics] = useState([]);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: rosbridgeUrl,
    });

    ros.on('connection', () => {
      setConnected(true);
      ros.getTopics((topicList) => {
        setTopics(topicList.topics);
      });
    });

    ros.on('error', () => {
      setConnected(false);
      setTopics([]);
    });

    ros.on('close', () => {
      setConnected(false);
      setTopics([]);
    });

    return () => {
      ros.close();
    };
  }, [rosbridgeUrl]);

  return (
    <div
      style={{
        background: '#fff',
        borderRadius: 16,
        boxShadow: '0 2px 8px rgba(0,0,0,0.06)',
        padding: 20,
        minWidth: 260,
        maxHeight: 400,
        overflowY: 'auto',
        margin: 16,
      }}
    >
      <div style={{ fontWeight: 600, marginBottom: 8 }}>ROS Topics</div>
      <div style={{ fontSize: 14, color: connected ? '#1a7f37' : '#c00', marginBottom: 8 }}>
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <ul style={{ padding: 0, margin: 0, listStyle: 'none' }}>
        {topics.map((topic) => (
          <li key={topic} style={{ padding: '2px 0', fontSize: 13 }}>
            {topic}
          </li>
        ))}
      </ul>
    </div>
  );
}
