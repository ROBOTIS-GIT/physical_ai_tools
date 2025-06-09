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

import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import ImageGridCell from './ImageGridCell';
import ImageTopicSelectModal from './ImageTopicSelectModal';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const layout = [{ aspect: '16/9' }, { aspect: '16/9' }, { aspect: '16/9' }];

export default function ImageGrid({ topics, setTopics, rosHost }) {
  const [modalOpen, setModalOpen] = React.useState(false);
  const [selectedIdx, setSelectedIdx] = React.useState(null);
  const [topicList, setTopicList] = useState([]);
  const [isLoadingTopics, setIsLoadingTopics] = useState(false);
  const [topicListError, setTopicListError] = useState(null);

  const rosbridgeUrl = `ws://${rosHost.split(':')[0]}:9090`;
  const { getImageTopicList } = useRosServiceCaller(rosbridgeUrl);

  // Adjust the length of the topics array
  React.useEffect(() => {
    if (topics.length !== layout.length) {
      setTopics(Array(layout.length).fill(null));
    }
    // eslint-disable-next-line
  }, []);

  // Fetch image topic list on component mount
  useEffect(() => {
    const fetchTopicList = async () => {
      setIsLoadingTopics(true);
      setTopicListError(null);
      try {
        const result = await getImageTopicList();
        if (result && result.success) {
          setTopicList(result.image_topic_list || []);
          setTopicListError(null);
          toast.success(`Loaded ${result.image_topic_list?.length || 0} image topics`);
        } else {
          console.error('Failed to get image topic list:', result?.message);
          const errorMsg = result?.message || 'Unknown error occurred';
          setTopicListError(`Service error: ${errorMsg}`);
          setTopicList([]);
          toast.error(`Failed to load image topics: ${errorMsg}`);
        }
      } catch (error) {
        console.error('Error fetching image topic list:', error);
        setTopicListError('Failed to load image topic list');
        setTopicList([]);
        toast.error('Failed to load image topic list');
      } finally {
        setIsLoadingTopics(false);
      }
    };

    fetchTopicList();
  }, [getImageTopicList]);

  const handlePlusClick = (idx) => {
    setSelectedIdx(idx);
    setModalOpen(true);
  };

  const handleRefreshTopics = async () => {
    setIsLoadingTopics(true);
    setTopicListError(null);
    try {
      const result = await getImageTopicList();
      if (result && result.success) {
        setTopicList(result.image_topic_list || []);
        setTopicListError(null);
        toast.success(`Refreshed: ${result.image_topic_list?.length || 0} image topics`);
      } else {
        const errorMsg = result?.message || 'Unknown error occurred';
        setTopicListError(`Service error: ${errorMsg}`);
        setTopicList([]);
        toast.error(`Failed to refresh topics: ${errorMsg}`);
      }
    } catch (error) {
      setTopicListError('Failed to load image topic list');
      setTopicList([]);
      toast.error('Failed to refresh image topics');
    } finally {
      setIsLoadingTopics(false);
    }
  };

  const handleTopicSelect = (topic) => {
    setTopics(topics.map((t, i) => (i === selectedIdx ? topic : t)));
    setModalOpen(false);
    setSelectedIdx(null);
  };

  const handleCellClose = (idx) => {
    const img = document.querySelector(`#img-stream-${idx}`);
    if (img) img.src = '';
    setTopics(topics.map((t, i) => (i === idx ? null : t)));
  };

  const classImageGridArea = clsx(
    'flex',
    'flex-row',
    'justify-center',
    'items-center',
    'gap-[0.5vw]',
    'w-full',
    'h-full',
    'max-w-full',
    'max-h-full',
    'overflow-hidden'
  );

  const classImageGridCell = (idx) =>
    clsx('min-w-0', 'min-h-0', 'flex', 'items-center', 'justify-center', 'relative', {
      'flex-[7_1_0]': idx === 1,
      'flex-[3_1_0]': idx !== 1,
    });

  const classTopicLabel = clsx(
    'absolute',
    'bottom-2',
    'left-2',
    'text-xs',
    'text-white',
    'bg-black',
    'bg-opacity-50',
    'px-2',
    'py-1',
    'rounded'
  );

  return (
    <div className="w-full h-full overflow-hidden">
      <div className={classImageGridArea}>
        {layout.map((cell, idx) => (
          <div key={idx} className={classImageGridCell(idx)}>
            <ImageGridCell
              topic={topics[idx]}
              aspect={cell.aspect}
              idx={idx}
              rosHost={rosHost}
              onClose={handleCellClose}
              onPlusClick={handlePlusClick}
            />
            <div className={classTopicLabel}>{topics[idx] || ''}</div>
          </div>
        ))}
        {modalOpen && (
          <ImageTopicSelectModal
            topicList={topicList}
            onSelect={handleTopicSelect}
            onClose={() => setModalOpen(false)}
            isLoading={isLoadingTopics}
            onRefresh={handleRefreshTopics}
            errorMessage={topicListError}
          />
        )}
      </div>
    </div>
  );
}
