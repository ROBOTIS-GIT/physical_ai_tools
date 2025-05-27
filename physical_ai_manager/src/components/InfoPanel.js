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
import './InfoPanel.css';

const InfoPanel = ({ info }) => (
  <div className="info-panel">
    <div>
      <span className="label">task name</span> <span className="value">{info.taskName}</span>
    </div>
    <div>
      <span className="label">robot type</span> <span className="value">{info.robotType}</span>
    </div>
    <div>
      <span className="label">task type</span> <span className="value">{info.taskType}</span>
    </div>
    <div className="info-panel-row">
      <span className="label" style={{ whiteSpace: 'nowrap' }}>
        single task
      </span>
      <span className="value" style={{ fontSize: '18px' }}>
        {info.singleTask}
      </span>
    </div>
    <div>
      <span className="label">fps</span> <span className="value">{info.fps}</span>
    </div>
    <div>
      <span className="label">tags</span> <span className="value">{JSON.stringify(info.tags)}</span>
    </div>
    <div>
      <span className="label">warmup time</span> <span className="value">{info.warmupTime}</span>
    </div>
    <div>
      <span className="label">episode time</span> <span className="value">{info.episodeTime}</span>
    </div>
    <div>
      <span className="label">reset time</span> <span className="value">{info.resetTime}</span>
    </div>
    <div>
      <span className="label">num episodes</span> <span className="value">{info.numEpisodes}</span>
    </div>
    <div>
      <span className="label">resume</span>{' '}
      <span className="value">{info.resume ? 'true' : 'false'}</span>
    </div>
    <div>
      <span className="label">push to hub</span>{' '}
      <span className="value">{info.pushToHub ? 'true' : 'false'}</span>
    </div>
  </div>
);

export default InfoPanel;
