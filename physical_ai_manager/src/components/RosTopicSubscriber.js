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

export default function RosTopicSubscriber({ rosbridgeUrl, topicName, messageType }) {
  const [message, setMessage] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: rosbridgeUrl });
    let topic = null;

    ros.on('connection', () => {
      setConnected(true);
      topic = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: messageType,
      });
      topic.subscribe((msg) => {
        setMessage(msg);
      });
    });
    ros.on('error', () => setConnected(false));
    ros.on('close', () => setConnected(false));

    return () => {
      if (topic) topic.unsubscribe();
      ros.close();
    };
  }, [rosbridgeUrl, topicName, messageType]);

  return (
    <div
      style={{
        background: '#fff',
        borderRadius: 16,
        boxShadow: '0 2px 8px rgba(0,0,0,0.06)',
        padding: 20,
        minWidth: 260,
        margin: 16,
      }}
    >
      <div style={{ fontWeight: 600, marginBottom: 8 }}>ROS Topic Subscribe</div>
      <div style={{ fontSize: 14, color: connected ? '#1a7f37' : '#c00', marginBottom: 8 }}>
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <div style={{ fontSize: 13, color: '#333', wordBreak: 'break-all' }}>
        <b>Topic:</b> {topicName}
        <br />
        <b>Type:</b> {messageType}
        <br />
        <b>Latest:</b>
        <br />
        <pre
          style={{
            fontSize: 12,
            background: '#f7f7f7',
            padding: 8,
            borderRadius: 8,
            maxHeight: 200,
            overflow: 'auto',
          }}
        >
          {message ? JSON.stringify(message, null, 2) : '(no data)'}
        </pre>
      </div>
    </div>
  );
}
