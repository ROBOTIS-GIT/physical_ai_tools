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
import { MdHome } from 'react-icons/md';
import { Toaster } from 'react-hot-toast';
import './App.css';
import HomePage from './pages/HomePage';
import SettingPage from './pages/SettingPage';

function App() {
  const defaultRosHost = window.location.hostname + ':8080';
  const [page, setPage] = useState('home');
  const [topics, setTopics] = useState([null, null, null, null]);
  const [rosHost, setRosHost] = useState(defaultRosHost);

  // Load YAML content from local storage
  const [yamlContent, setYamlContent] = useState(() => {
    const savedContent = localStorage.getItem('yamlFileContent');
    try {
      return savedContent ? JSON.parse(savedContent) : null;
    } catch (error) {
      console.error('Error parsing YAML data from local storage:', error);
      return null;
    }
  });

  // Debug log for yamlContent
  useEffect(() => {
    console.log('App.js - yamlContent state:', yamlContent);
  }, [yamlContent]);

  return (
    <div className="flex h-screen w-screen">
      <aside className="w-30 bg-gray-200 h-full flex flex-col items-center pt-10 gap-6">
        <button
          className={clsx(
            'flex',
            'flex-col',
            'items-center',
            'bg-gray-100',
            'rounded-2xl',
            'border-none',
            'py-5',
            'px-4',
            'mb-3',
            'text-base',
            'text-gray-800',
            'cursor-pointer',
            'transition-colors',
            'duration-150',
            'outline-none',
            'min-w-20',
            {
              'hover:bg-gray-300 active:bg-gray-400': page !== 'home',
              'bg-gray-300': page === 'home',
            }
          )}
          onClick={() => setPage('home')}
        >
          <MdHome size={32} className="mb-1.5" />
          <span className="mt-1 text-sm">Home</span>
        </button>
      </aside>
      <main className="flex-1 flex flex-col h-screen min-h-0">
        {page === 'home' ? (
          <HomePage
            topics={topics}
            setTopics={setTopics}
            rosHost={rosHost}
            yamlContent={yamlContent}
          />
        ) : (
          <SettingPage
            rosHost={rosHost}
            setRosHost={setRosHost}
            yamlContent={yamlContent}
            setYamlContent={setYamlContent}
          />
        )}
      </main>
      <Toaster
        position="top-center"
        gutter={8}
        toastOptions={{
          duration: 3000,
          style: {
            background: '#363636',
            color: '#fff',
          },
          success: {
            duration: 2000,
            style: {
              background: '#10b981',
            },
          },
          error: {
            duration: 4000,
            style: {
              background: '#ef4444',
            },
          },
        }}
      />
    </div>
  );
}

export default App;
