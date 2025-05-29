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
import clsx from 'clsx';
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
    <div className="bg-white rounded-lg shadow-md p-6 w-full max-w-md">
      <h3 className="text-lg font-semibold mb-4">ROS Topic Subscriber</h3>
      <div
        className={clsx('text-sm mb-2', {
          'text-green-700': connected,
          'text-red-600': !connected,
        })}
      >
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <div className="text-xs text-gray-800 break-all">
        <b>Topic:</b> {topicName}
        <br />
        <b>Type:</b> {messageType}
        <br />
        <b>Latest:</b>
        <br />
        <pre className="text-xs bg-gray-100 p-2 rounded-lg max-h-50 overflow-auto">
          {message ? JSON.stringify(message, null, 2) : '(no data)'}
        </pre>
      </div>
    </div>
  );
}
