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

export default function ProgressBar({ percent = 0 }) {
  // If over 50%, white, otherwise dark color
  const textColor = percent > 50 ? '#fff' : '#222';
  return (
    <div
      style={{
        width: '100%',
        maxWidth: 400,
        height: 38,
        background: '#e0e0e0',
        borderRadius: 20,
        position: 'relative',
        overflow: 'hidden',
      }}
    >
      <div
        style={{
          width: `${percent}%`,
          height: '100%',
          background: '#444',
          borderRadius: 20,
          transition: 'width 0.3s',
        }}
      ></div>
      <span
        style={{
          position: 'absolute',
          left: 0,
          top: 0,
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: textColor,
          fontWeight: 700,
          fontSize: 18,
          pointerEvents: 'none',
          zIndex: 2,
          transition: 'color 0.3s',
        }}
      >
        {percent}%
      </span>
    </div>
  );
}
