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

import React, { useState, useCallback } from 'react';
import RosTopicList from '../components/RosTopicList';
import RosTopicSubscriber from '../components/RosTopicSubscriber';

// Flux imports
import { useAppStore } from '../flux/hooks/useAppStore';
import AppActions from '../flux/actions/AppActions';

export default function SettingPage() {
  // Get state from Flux stores
  const { rosHost, yamlContent } = useAppStore();

  const [tempRosHost, setTempRosHost] = useState(rosHost);

  const handleRosHostSave = useCallback(() => {
    AppActions.setRosHost(tempRosHost);
  }, [tempRosHost]);

  const handleYamlUpload = useCallback((event) => {
    const file = event.target.files[0];
    if (file) {
      const reader = new FileReader();
      reader.onload = (e) => {
        try {
          const content = JSON.parse(e.target.result);
          AppActions.setYamlContent(content);
        } catch (error) {
          AppActions.setError('Invalid YAML file format');
        }
      };
      reader.readAsText(file);
    }
  }, []);

  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;

  return (
    <div className="p-6">
      <h1 className="text-2xl font-bold mb-6">Settings</h1>

      <div className="space-y-6">
        {/* ROS Host Settings */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">ROS Host</label>
          <div className="flex gap-2">
            <input
              type="text"
              value={tempRosHost}
              onChange={(e) => setTempRosHost(e.target.value)}
              className="flex-1 p-2 border border-gray-300 rounded-md"
              placeholder="localhost:8080"
            />
            <button
              onClick={handleRosHostSave}
              className="px-4 py-2 bg-blue-500 text-white rounded-md hover:bg-blue-600"
            >
              Save
            </button>
          </div>
        </div>

        {/* YAML Configuration */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">YAML Configuration</label>
          <input
            type="file"
            accept=".yaml,.yml,.json"
            onChange={handleYamlUpload}
            className="block w-full text-sm text-gray-500 file:mr-4 file:py-2 file:px-4 file:rounded-full file:border-0 file:text-sm file:font-semibold file:bg-blue-50 file:text-blue-700 hover:file:bg-blue-100"
          />
          {yamlContent && (
            <div className="mt-2 p-2 bg-gray-100 rounded-md">
              <pre className="text-xs text-gray-600">
                {JSON.stringify(yamlContent, null, 2).substring(0, 200)}...
              </pre>
            </div>
          )}
        </div>
      </div>

      <div className="w-full h-full flex items-start justify-center text-xl flex-row gap-8 mt-8">
        <div className="flex flex-col items-center mt-10 w-full">
          <div className="mb-6 text-3xl font-semibold">
            Image Streaming Server Address Configuration
          </div>
          <div className="text-gray-500 text-base">
            Current auto-set value: <b>{rosHost}</b>
            <br />
            (This value is automatically determined based on window.location.hostname)
          </div>
        </div>
        <div className="flex flex-col items-stretch">
          <RosTopicList rosbridgeUrl={rosbridgeUrl} />
          <RosTopicSubscriber
            rosbridgeUrl={rosbridgeUrl}
            topicName="/chatter"
            messageType="std_msgs/String"
          />
        </div>
      </div>
    </div>
  );
}
