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

export default function EpisodeStatus() {
  return (
    <div
      style={{
        height: '100%',
        width: '300px',
        textAlign: 'center',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        borderRadius: 16,
        border: '0px solid transparent',
        padding: 16,
        boxSizing: 'border-box',
        boxShadow: 'rgba(0, 0, 0, 0.06) 0px 2px 8px',
      }}
    >
      <div style={{ fontSize: 32, marginBottom: 16, justifyContent: 'flex-start' }}>Episode</div>
      <div
        style={{
          background: '#ededed',
          borderRadius: 12,
          padding: '16px 32px',
          fontSize: 24,
          fontWeight: 'bold',
        }}
      >
        <span style={{ fontWeight: 700 }}>14</span> / <span style={{ color: '#555' }}>150</span>
      </div>
    </div>
  );
}
