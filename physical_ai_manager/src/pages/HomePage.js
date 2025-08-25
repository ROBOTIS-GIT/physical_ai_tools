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
import clsx from 'clsx';
import RobotTypeSelector from '../components/RobotTypeSelector';
import HeartbeatStatus from '../components/HeartbeatStatus';
import packageJson from '../../package.json';
import { FaGithub, FaYoutube } from 'react-icons/fa';
import { SiHuggingface } from 'react-icons/si';
import { FaExternalLinkAlt } from 'react-icons/fa';

export default function HomePage() {
  const classContainer = clsx(
    'w-full',
    'h-full',
    'flex',
    'items-center',
    'justify-center',
    'pt-10'
  );

  const classHeartbeatStatus = clsx('absolute', 'top-5', 'left-35', 'z-10');

  const classLinkItem = (icon, link, text) => {
    return (
      <div className="flex flex-row items-center justify-center gap-2">
        {icon}
        <div>
          <a
            href={link}
            target="_blank"
            rel="noopener noreferrer"
            className="flex items-center gap-1 hover:text-blue-600 transition-colors"
          >
            {text}
          </a>
        </div>
      </div>
    );
  };

  const aiRobotisDotCom = () => {
    return classLinkItem(
      <FaExternalLinkAlt className="text-xl" />,
      'https://ai.robotis.com',
      'ai.robotis.com'
    );
  };

  const youtubeRobotisOpenSourceTeam = () => {
    return classLinkItem(
      <FaYoutube className="text-xl" />,
      'https://www.youtube.com/@ROBOTISOpenSourceTeam',
      'Youtube@ROBOTISOpenSourceTeam'
    );
  };

  const githubRobotisGitPhysicalAiTools = () => {
    return classLinkItem(
      <FaGithub className="text-xl" />,
      'https://github.com/ROBOTIS-GIT/physical_ai_tools',
      'ROBOTIS-GIT/physical_ai_tools'
    );
  };

  const huggingfaceROBOTIS = () => {
    return classLinkItem(
      <SiHuggingface className="text-xl text-gray-700 rounded-md" />,
      'https://huggingface.co/ROBOTIS',
      'huggingface.co/ROBOTIS'
    );
  };

  const aboutPhysicalAiManager = () => {
    return (
      <div className="flex flex-col items-center justify-center m-5 gap-5 min-w-72">
        <p className="text-3xl font-bold">Physical AI Manager</p>
        <div className="flex flex-col items-center justify-center gap-2">
          <div className="flex flex-col items-center justify-center">
            <div>
              <p>{packageJson.description}</p>
            </div>
            <div className="flex flex-row items-center justify-center gap-2">
              <p className="font-semibold">Version</p>
              <p className="bg-blue-400 text-white rounded-2xl font-semibold px-2 py-1 shadow-md">
                {packageJson.version}
              </p>
            </div>
            <span className="h-6" />
            <div className="flex flex-col items-center justify-center bg-gray-100 rounded-lg p-4 shadow-md">
              <p className="text-lg font-bold">Quick Links</p>
              <span className="h-2" />
              {aiRobotisDotCom()}
              {youtubeRobotisOpenSourceTeam()}
              {githubRobotisGitPhysicalAiTools()}
              {huggingfaceROBOTIS()}
            </div>
          </div>
        </div>
      </div>
    );
  };

  return (
    <div className={classContainer}>
      <div className={classHeartbeatStatus}>
        <HeartbeatStatus />
      </div>
      <div className="flex flex-raw items-center justify-center gap-16">
        {aboutPhysicalAiManager()}
        <RobotTypeSelector />
      </div>
    </div>
  );
}
