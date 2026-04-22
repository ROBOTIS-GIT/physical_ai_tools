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

import React, { useState } from 'react';
import clsx from 'clsx';
import { useSelector } from 'react-redux';
import { MdRefresh } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const STATUS_OK = 0;
const STATUS_SLOW = 1;
const STATUS_STALLED = 2;

const statusColor = {
  [STATUS_OK]: 'bg-green-500',
  [STATUS_SLOW]: 'bg-yellow-400',
  [STATUS_STALLED]: 'bg-red-500',
};

const statusLabel = {
  [STATUS_OK]: 'OK',
  [STATUS_SLOW]: 'Slow',
  [STATUS_STALLED]: 'Stalled',
};

const shortenTopic = (name) => {
  const parts = (name || '').split('/').filter(Boolean);
  if (parts.length <= 2) return name;
  return '.../' + parts.slice(-2).join('/');
};

export default function RecordTopicMonitor() {
  const monitor = useSelector((state) => state.tasks.recordingMonitor);
  const { sendRecordCommand } = useRosServiceCaller();
  const [isRefreshing, setIsRefreshing] = useState(false);

  const handleRefresh = async () => {
    if (isRefreshing) return;
    setIsRefreshing(true);
    try {
      await sendRecordCommand('refresh_topics');
    } catch (_) {
      // ignore; spinner will clear
    } finally {
      // Small delay so the spinner is visible and users see the reset.
      setTimeout(() => setIsRefreshing(false), 400);
    }
  };

  if (!monitor?.topics?.length) return null;

  const sorted = [...monitor.topics].sort((a, b) => b.status - a.status);
  const problemCount = monitor.topics.filter((t) => t.status !== STATUS_OK).length;
  const worstStatus = monitor.topics.reduce((max, t) => Math.max(max, t.status), STATUS_OK);
  const overallColor = statusColor[worstStatus] || 'bg-green-500';

  return (
    <div className="bg-white border border-gray-200 rounded-2xl shadow-md p-3 w-full h-full overflow-y-auto">
      <div className="flex items-center gap-2 mb-2">
        <span className={clsx('inline-block w-3 h-3 rounded-full', overallColor)} />
        <span className="text-sm font-semibold text-gray-800">Topic Monitor</span>
        <span className="text-xs text-gray-400 ml-auto">
          {problemCount > 0
            ? `${problemCount} issue(s)`
            : `All ${monitor.topics.length} OK`}
        </span>
        <button
          type="button"
          onClick={handleRefresh}
          disabled={isRefreshing}
          title="Reset topic subscriptions"
          className={clsx(
            'p-1 rounded-md text-gray-500 hover:bg-gray-100 hover:text-gray-700',
            'disabled:text-gray-300 disabled:cursor-not-allowed'
          )}
        >
          <MdRefresh className={clsx('w-4 h-4', isRefreshing && 'animate-spin')} />
        </button>
      </div>
      <table className="w-full text-xs">
        <thead>
          <tr className="text-gray-500 border-b border-gray-100">
            <th className="text-left py-1 font-medium">Topic</th>
            <th className="text-right py-1 font-medium w-14">Hz</th>
            <th className="text-center py-1 font-medium w-12">Status</th>
          </tr>
        </thead>
        <tbody>
          {sorted.map((t) => (
            <tr key={t.name} className="border-b border-gray-50 last:border-0">
              <td className="py-1 truncate max-w-[160px]" title={t.name}>
                {shortenTopic(t.name)}
              </td>
              <td className="py-1 text-right font-mono text-gray-700">
                {t.rateHz.toFixed(1)}
              </td>
              <td className="py-1 text-center">
                <span
                  className={clsx(
                    'inline-block w-2.5 h-2.5 rounded-full',
                    statusColor[t.status] || 'bg-gray-300'
                  )}
                  title={`${statusLabel[t.status] || '?'} (baseline ${t.baselineHz.toFixed(1)} Hz, last ${t.secondsSinceLast >= 0 ? t.secondsSinceLast.toFixed(1) + 's ago' : 'never'})`}
                />
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
