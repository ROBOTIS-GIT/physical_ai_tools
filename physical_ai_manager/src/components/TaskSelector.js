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

import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';

export default function TaskSelector({ onTaskSelect, yamlContent }) {
  const [isOpen, setIsOpen] = useState(false);
  const [tasks, setTasks] = useState([]);
  const [selectedTask, setSelectedTask] = useState(null);
  const dropdownRef = useRef(null);

  useEffect(() => {
    // Get top-level categories from YAML
    const fetchTasks = () => {
      console.log('yamlContent Type:', typeof yamlContent);
      console.log('yamlContent Value:', yamlContent);

      if (yamlContent) {
        try {
          // Extract top-level keys from YAML content
          let topLevelCategories = [];

          if (typeof yamlContent === 'object' && yamlContent !== null) {
            topLevelCategories = Object.keys(yamlContent);
          } else if (typeof yamlContent === 'string') {
            // Try parsing if it's a string
            try {
              const parsedContent = JSON.parse(yamlContent);
              topLevelCategories = Object.keys(parsedContent);
            } catch (e) {
              console.error('String parsing failed:', e);
            }
          }

          console.log('Extracted categories:', topLevelCategories);
          setTasks(topLevelCategories);
        } catch (error) {
          console.error('Error processing YAML data:', error);
          setTasks([]);
        }
      } else {
        // If no YAML content, set empty array
        console.log('No YAML content');
        setTasks([]);
      }
    };

    fetchTasks();
  }, [yamlContent]);

  useEffect(() => {
    // Close dropdown when clicking outside
    function handleClickOutside(event) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    }

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleTaskClick = (task) => {
    setSelectedTask(task);
    setIsOpen(false);
    if (onTaskSelect) {
      onTaskSelect(task);
    }
  };

  return (
    <div className="mb-6 relative" ref={dropdownRef}>
      <div className="flex flex-col gap-2.5">
        <button
          className={clsx(
            'w-38 h-10 text-lg cursor-pointer bg-white border border-gray-300 rounded',
            'disabled:cursor-not-allowed disabled:bg-gray-100'
          )}
          onClick={() => setIsOpen(!isOpen)}
          disabled={tasks.length === 0}
        >
          {tasks.length === 0 ? 'No Tasks' : 'Select Task'}
        </button>
      </div>

      {isOpen && (
        <div
          className={clsx(
            'absolute',
            'mt-1',
            'bg-white',
            'border',
            'border-gray-300',
            'rounded-lg',
            'shadow-lg',
            'z-50',
            'max-h-75',
            'overflow-y-auto'
          )}
        >
          {tasks.map((task, index) => (
            <div
              key={index}
              className={clsx(
                'py-2.5 px-4 cursor-pointer border-b border-gray-200',
                'last:border-b-0 hover:bg-gray-100'
              )}
              onClick={() => handleTaskClick(task)}
            >
              {task}
            </div>
          ))}
        </div>
      )}

      {selectedTask && (
        <div className="mt-2.5 bg-gray-200 rounded-lg py-2 px-4 text-lg">{selectedTask}</div>
      )}
    </div>
  );
}
