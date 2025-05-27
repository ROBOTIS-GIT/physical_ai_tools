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

import React, { useState } from 'react';
import ROSLIB from 'roslib';

export default function RosServiceCaller({
  rosbridgeUrl,
  serviceName,
  serviceType,
  requestFields,
}) {
  const [response, setResponse] = useState(null);
  const [connected, setConnected] = useState(false);
  const [request, setRequest] = useState(requestFields || {});
  const [loading, setLoading] = useState(false);

  React.useEffect(() => {
    const ros = new ROSLIB.Ros({ url: rosbridgeUrl });
    ros.on('connection', () => setConnected(true));
    ros.on('error', () => setConnected(false));
    ros.on('close', () => setConnected(false));
    return () => ros.close();
  }, [rosbridgeUrl]);

  const handleChange = (e) => {
    setRequest({ ...request, [e.target.name]: e.target.value });
  };

  const callService = () => {
    setLoading(true);
    const ros = new ROSLIB.Ros({ url: rosbridgeUrl });
    const service = new ROSLIB.Service({
      ros,
      name: serviceName,
      serviceType: serviceType,
    });
    const req = new ROSLIB.ServiceRequest(request);
    service.callService(req, (result) => {
      setResponse(result);
      setLoading(false);
      ros.close();
    });
  };

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
      <div style={{ fontWeight: 600, marginBottom: 8 }}>ROS Service Call</div>
      <div style={{ fontSize: 14, color: connected ? '#1a7f37' : '#c00', marginBottom: 8 }}>
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <div style={{ fontSize: 13, color: '#333', wordBreak: 'break-all', marginBottom: 8 }}>
        <b>Service:</b> {serviceName}
        <br />
        <b>Type:</b> {serviceType}
      </div>
      <form
        onSubmit={(e) => {
          e.preventDefault();
          callService();
        }}
        style={{ marginBottom: 8 }}
      >
        {Object.keys(request).map((key) => (
          <div key={key} style={{ marginBottom: 4 }}>
            <label style={{ fontSize: 12 }}>{key}: </label>
            <input
              name={key}
              value={request[key]}
              onChange={handleChange}
              style={{
                fontSize: 12,
                padding: 2,
                borderRadius: 4,
                border: '1px solid #ccc',
                width: 120,
              }}
            />
          </div>
        ))}
        <button
          type="submit"
          disabled={loading}
          style={{
            marginTop: 8,
            padding: '4px 12px',
            borderRadius: 6,
            border: 'none',
            background: '#1a7f37',
            color: '#fff',
            fontWeight: 600,
            cursor: 'pointer',
          }}
        >
          {loading ? 'Calling...' : 'Call Service'}
        </button>
      </form>
      <div style={{ fontSize: 12, color: '#333', wordBreak: 'break-all' }}>
        <b>Response:</b>
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
          {response ? JSON.stringify(response, null, 2) : '(no response)'}
        </pre>
      </div>
    </div>
  );
}
