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
    <div className="bg-white rounded-lg shadow-md p-6 w-full max-w-md">
      <h3 className="text-lg font-semibold mb-4">ROS Topics</h3>
      <div
        className={clsx('text-sm mb-2', {
          'text-green-700': connected,
          'text-red-600': !connected,
        })}
      >
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <ul className="p-0 m-0 list-none">
        {topics.map((topic) => (
          <li key={topic} className="py-0.5 text-xs">
            {topic}
          </li>
        ))}
      </ul>
    </div>
  );
}
