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
import ProgressBar from './ProgressBar';
import { MdPlayArrow, MdStop, MdReplay, MdSkipNext, MdCheck } from 'react-icons/md';

const panelStyle = {
  height: 200,
  background: '#bdbdbd',
  borderRadius: 27,
  margin: 30,
  padding: 14,
  display: 'flex',
  flexDirection: 'row',
  alignItems: 'center',
  gap: 8,
  boxShadow: 'rgba(0, 0, 0, 0.1) 0px 2px 8px',
};

const buttonStyle = {
  fontSize: '40px',
  fontFamily: 'Pretendard Variable',
  fontWeight: 800,
  height: '100%',
  flexGrow: 1,
  minWidth: 60,
  borderRadius: 20,
  border: 'none',
  background: '#ededed',
  cursor: 'pointer',
  marginRight: 8,
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  flexDirection: 'column',
  gap: 4,
};

const iconWrapperStyle = {
  background: '#ffffffbb',
  borderRadius: '50%',
  width: 56,
  height: 56,
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  marginBottom: 4,
};

const buttons = [
  { label: 'Start', icon: MdPlayArrow, color: '#1976d2' },
  { label: 'Stop', icon: MdStop, color: '#d32f2f' },
  { label: 'Retry', icon: MdReplay, color: '#fbc02d' },
  { label: 'Next', icon: MdSkipNext, color: '#388e3c' },
  { label: 'Finish', icon: MdCheck, color: '#388e3c' },
];

export default function ControlPanel() {
  const icon_size = 40;
  return (
    <div className="control-panel-fixed" style={panelStyle}>
      <div
        style={{
          display: 'flex',
          flex: 2,
          alignItems: 'center',
          width: '100%',
          height: '100%',
          gap: 16,
        }}
      >
        {buttons.map(({ label, icon: Icon, color }) => (
          <button key={label} style={buttonStyle}>
            <span style={iconWrapperStyle}>
              <Icon size={icon_size} color={color} />
            </span>
            {label}
          </button>
        ))}
      </div>
      <div
        style={{
          display: 'flex',
          flex: 1,
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          width: '100%',
          gap: 30,
        }}
      >
        <div style={{ flex: 1, minWidth: 0, fontSize: 33, textAlign: 'center' }}>
          Please prepare next episode
        </div>
        <ProgressBar percent={60} />
      </div>
    </div>
  );
}
