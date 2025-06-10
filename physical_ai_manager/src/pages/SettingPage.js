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

import React from 'react';
import YamlEditor from '../components/YamlEditor';
import RosTopicList from '../components/RosTopicList';
import RosTopicSubscriber from '../components/RosTopicSubscriber';
import RosServiceCaller from '../components/RosServiceCaller';

export default function SettingPage({ rosHost, setRosHost, yamlContent, setYamlContent }) {
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
      </div>
    </div>
  );
}
