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

import React, { useState, useImperativeHandle, forwardRef } from 'react';
import clsx from 'clsx';
import ROSLIB from 'roslib';

const RosServiceCaller = forwardRef(function RosServiceCaller(
  { rosbridgeUrl, serviceName, serviceType, requestFields },
  ref
) {
  const [response, setResponse] = useState(null);
  const [connected, setConnected] = useState(false);
  const [request, setRequest] = useState(requestFields || {});
  const [loading, setLoading] = useState(false);

  React.useEffect(() => {
    const ros = new ROSLIB.Ros({ url: rosbridgeUrl });

    ros.on('connection', () => {
      console.log('RosServiceCaller: Connected to ROS bridge');
      setConnected(true);
    });

    ros.on('error', (error) => {
      console.error('RosServiceCaller: ROS connection error:', error);
      setConnected(false);
    });

    ros.on('close', () => {
      console.log('RosServiceCaller: ROS connection closed');
      setConnected(false);
    });

    return () => {
      try {
        ros.close();
      } catch (error) {
        console.error('Error closing ROS connection:', error);
      }
    };
  }, [rosbridgeUrl]);

  const handleChange = (e) => {
    setRequest({ ...request, [e.target.name]: e.target.value });
  };

  const callService = (serviceName, serviceType, request) => {
    if (!connected) {
      console.warn('ROS not connected, cannot call service');
      setResponse({ error: 'ROS not connected' });
      return;
    }

    setLoading(true);
    setResponse(null);

    try {
      const ros = new ROSLIB.Ros({ url: rosbridgeUrl });

      ros.on('connection', () => {
        const service = new ROSLIB.Service({
          ros,
          name: serviceName,
          serviceType: serviceType,
        });
        const req = new ROSLIB.ServiceRequest(request);

        service.callService(
          req,
          (result) => {
            console.log('Service call successful:', result);
            setResponse(result);
            setLoading(false);
            ros.close();
          },
          (error) => {
            console.error('Service call failed:', error);
            setResponse({ error: error.toString() });
            setLoading(false);
            ros.close();
          }
        );
      });

      ros.on('error', (error) => {
        console.error('ROS connection error during service call:', error);
        setResponse({ error: 'Connection failed: ' + error.toString() });
        setLoading(false);
        ros.close();
      });
    } catch (error) {
      console.error('Failed to create ROS connection for service call:', error);
      setResponse({ error: 'Failed to create connection: ' + error.toString() });
      setLoading(false);
    }
  };

  useImperativeHandle(ref, () => ({
    callService,
  }));

  return (
    <div className="bg-white rounded-lg shadow-md p-6 w-full max-w-md">
      <h3 className="text-lg font-semibold mb-4">ROS Service Caller</h3>
      <div
        className={clsx('text-sm mb-2', {
          'text-green-700': connected,
          'text-red-600': !connected,
        })}
      >
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <div className="text-xs text-gray-800 break-all mb-2">
        <b>Service:</b> {serviceName}
        <br />
        <b>Type:</b> {serviceType}
      </div>
      <form
        onSubmit={(e) => {
          e.preventDefault();
          callService();
        }}
        className="mb-2"
      >
        {Object.keys(request).map((key) => (
          <div key={key} className="mb-1">
            <label className="text-xs">{key}: </label>
            <input
              name={key}
              value={request[key]}
              onChange={handleChange}
              className={clsx('text-xs', 'p-0.5', 'rounded', 'border', 'border-gray-300', 'w-30')}
            />
          </div>
        ))}
        <button
          type="submit"
          disabled={loading}
          className={clsx(
            'mt-2',
            'py-1',
            'px-3',
            'rounded-md',
            'border-none',
            'bg-green-700',
            'text-white',
            'font-semibold',
            'cursor-pointer',
            'disabled:opacity-50'
          )}
        >
          {loading ? 'Calling...' : 'Call Service'}
        </button>
      </form>
      <div className="text-xs text-gray-800 break-all">
        <b>Response:</b>
        <br />
        <pre className="text-xs bg-gray-100 p-2 rounded-lg max-h-50 overflow-auto">
          {response ? JSON.stringify(response, null, 2) : '(no response)'}
        </pre>
      </div>
    </div>
  );
});

export default RosServiceCaller;
