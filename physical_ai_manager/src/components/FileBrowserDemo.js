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

import React, { useState, useCallback } from 'react';
import { MdFolderOpen, MdInsertDriveFile } from 'react-icons/md';
import FileBrowser from './FileBrowser';
import FileBrowserModal from './FileBrowserModal';

export default function FileBrowserDemo() {
  const [selectedFile, setSelectedFile] = useState(null);
  const [currentPath, setCurrentPath] = useState('');
  const [showModal, setShowModal] = useState(false);
  const [modalSelectedFile, setModalSelectedFile] = useState(null);

  // File filter examples
  const imageFilter = useCallback((item) => {
    const imageExtensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp', '.webp'];
    return imageExtensions.some((ext) => item.name.toLowerCase().endsWith(ext));
  }, []);

  const configFilter = useCallback((item) => {
    const configExtensions = ['.yaml', '.yml', '.json', '.xml', '.cfg', '.conf'];
    return configExtensions.some((ext) => item.name.toLowerCase().endsWith(ext));
  }, []);

  const handleFileSelect = useCallback((file) => {
    setSelectedFile(file);
    console.log('File selected:', file);
  }, []);

  const handlePathChange = useCallback((path) => {
    setCurrentPath(path);
    console.log('Path changed:', path);
  }, []);

  const handleModalFileSelect = useCallback((file) => {
    setModalSelectedFile(file);
    console.log('Modal file selected:', file);
  }, []);

  return (
    <div className="p-6 space-y-8">
      <div className="max-w-6xl mx-auto">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">File Browser Demo</h1>
        <p className="text-gray-600 mb-8">
          Examples of using the FileBrowser and FileBrowserModal components.
        </p>

        {/* Basic File Browser */}
        <div className="bg-white rounded-lg shadow-lg p-6 mb-8">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">Basic File Browser</h2>
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <div className="lg:col-span-2">
              <FileBrowser
                onFileSelect={handleFileSelect}
                onPathChange={handlePathChange}
                className="h-96"
                title="Browse Files"
              />
            </div>
            <div className="space-y-4">
              <div className="bg-gray-50 p-4 rounded-lg">
                <h3 className="font-medium text-gray-900 mb-2">Selected File</h3>
                {selectedFile ? (
                  <div className="space-y-2">
                    <div className="flex items-center">
                      {selectedFile.is_directory ? (
                        <MdFolderOpen className="w-4 h-4 text-blue-500 mr-2" />
                      ) : (
                        <MdInsertDriveFile className="w-4 h-4 text-gray-400 mr-2" />
                      )}
                      <span className="text-sm font-medium">{selectedFile.name}</span>
                    </div>
                    <div className="text-xs text-gray-500">
                      <p>Path: {selectedFile.full_path}</p>
                      <p>
                        Size: {selectedFile.size >= 0 ? `${selectedFile.size} bytes` : 'Directory'}
                      </p>
                      <p>Modified: {selectedFile.modified_time}</p>
                    </div>
                  </div>
                ) : (
                  <p className="text-gray-500 text-sm">No file selected</p>
                )}
              </div>

              <div className="bg-gray-50 p-4 rounded-lg">
                <h3 className="font-medium text-gray-900 mb-2">Current Path</h3>
                <p className="text-sm font-mono text-gray-600 break-all">{currentPath || '/'}</p>
              </div>
            </div>
          </div>
        </div>

        {/* Modal Examples */}
        <div className="bg-white rounded-lg shadow-lg p-6 mb-8">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">File Browser Modal Examples</h2>
          <p className="text-gray-600 mb-4">
            These examples show different ways to configure the file browser modal, including custom
            home paths and file filtering options.
          </p>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
            <button
              onClick={() => setShowModal('basic')}
              className="p-4 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
            >
              Basic File Selection
            </button>
            <button
              onClick={() => setShowModal('images')}
              className="p-4 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors"
            >
              Image Files Only
            </button>
            <button
              onClick={() => setShowModal('config')}
              className="p-4 bg-purple-600 text-white rounded-lg hover:bg-purple-700 transition-colors"
            >
              Config Files Only
            </button>
            <button
              onClick={() => setShowModal('directory')}
              className="p-4 bg-orange-600 text-white rounded-lg hover:bg-orange-700 transition-colors"
            >
              Directory Selection
            </button>
          </div>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-4">
            <button
              onClick={() => setShowModal('ros_package')}
              className="p-4 bg-emerald-600 text-white rounded-lg hover:bg-emerald-700 transition-colors"
            >
              Find ROS Package Folder
            </button>
            <button
              onClick={() => setShowModal('cmake_project')}
              className="p-4 bg-teal-600 text-white rounded-lg hover:bg-teal-700 transition-colors"
            >
              Find CMake Project Folder
            </button>
          </div>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-4">
            <button
              onClick={() => setShowModal('workspace_home')}
              className="p-4 bg-indigo-600 text-white rounded-lg hover:bg-indigo-700 transition-colors"
            >
              Browse with Workspace Home
            </button>
            <button
              onClick={() => setShowModal('project_home')}
              className="p-4 bg-violet-600 text-white rounded-lg hover:bg-violet-700 transition-colors"
            >
              Browse with Project Home
            </button>
          </div>

          {modalSelectedFile && (
            <div className="bg-blue-50 border border-blue-200 rounded-lg p-4">
              <h3 className="font-medium text-blue-900 mb-2">Last Selected from Modal</h3>
              <div className="flex items-center">
                {modalSelectedFile.is_directory ? (
                  <MdFolderOpen className="w-4 h-4 text-blue-500 mr-2" />
                ) : (
                  <MdInsertDriveFile className="w-4 h-4 text-gray-400 mr-2" />
                )}
                <span className="text-sm font-medium text-blue-800">{modalSelectedFile.name}</span>
              </div>
              <p className="text-xs text-blue-600 mt-1 font-mono">{modalSelectedFile.full_path}</p>
            </div>
          )}
        </div>

        {/* File Browser with Filter Example */}
        <div className="bg-white rounded-lg shadow-lg p-6 mb-8">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">Image Files Browser</h2>
          <p className="text-gray-600 mb-4">
            This browser only shows directories and image files (.jpg, .png, .gif, etc.)
          </p>
          <FileBrowser fileFilter={imageFilter} className="h-80" title="Image Files Only" />
        </div>

        {/* Target File Search Example */}
        <div className="bg-white rounded-lg shadow-lg p-6">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">
            Find Directories Containing Specific Files
          </h2>
          <p className="text-gray-600 mb-6">
            When <code className="bg-gray-100 px-1 rounded">targetFileName</code> is specified, only
            directories are shown (files are hidden). You can customize the display label using{' '}
            <code className="bg-gray-100 px-1 rounded">targetFileLabel</code> prop instead of the
            default "Contains filename" text.
          </p>
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <div>
              <h3 className="text-lg font-medium text-gray-900 mb-2">Find package.xml files</h3>
              <p className="text-gray-600 mb-4">
                Only directories are shown when looking for specific files. Directories containing
                package.xml will be highlighted in green. Click on a green directory to select it.
              </p>
              <FileBrowser
                targetFileName="package.xml"
                targetFileLabel="📦 ROS Package"
                className="h-80"
                title="Find ROS Packages"
                onDirectorySelect={(dir) => {
                  console.log('ROS package directory selected:', dir);
                  setModalSelectedFile(dir);
                }}
              />
            </div>
            <div>
              <h3 className="text-lg font-medium text-gray-900 mb-2">Find CMakeLists.txt files</h3>
              <p className="text-gray-600 mb-4">
                Only directories are shown when looking for specific files. Directories containing
                CMakeLists.txt will be highlighted in green. This is useful for finding CMake
                projects.
              </p>
              <FileBrowser
                targetFileName="CMakeLists.txt"
                targetFileLabel="🔧 CMake Project"
                className="h-80"
                title="Find CMake Projects"
                onDirectorySelect={(dir) => {
                  console.log('CMake project directory selected:', dir);
                  setModalSelectedFile(dir);
                }}
              />
            </div>
          </div>
        </div>

        {/* Custom Home Path Examples */}
        <div className="bg-white rounded-lg shadow-lg p-6 mb-8">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">Custom Home Path Examples</h2>
          <p className="text-gray-600 mb-6">
            You can customize where the home button takes you using the{' '}
            <code className="bg-gray-100 px-1 rounded">homePath</code> prop. This is useful for
            setting a project root or specific starting directory.
          </p>
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <div>
              <h3 className="text-md font-medium text-gray-900 mb-2">Default Home</h3>
              <p className="text-sm text-gray-600 mb-3">
                System home directory (no homePath specified)
              </p>
              <FileBrowser className="h-64" title="Default Home" />
            </div>
            <div>
              <h3 className="text-md font-medium text-gray-900 mb-2">Workspace Root</h3>
              <p className="text-sm text-gray-600 mb-3">Set to workspace root directory</p>
              <FileBrowser homePath="/home/ola/ai_ws" className="h-64" title="Workspace Browser" />
            </div>
            <div>
              <h3 className="text-md font-medium text-gray-900 mb-2">Project Source</h3>
              <p className="text-sm text-gray-600 mb-3">Set to specific project source directory</p>
              <FileBrowser homePath="/home/ola/ai_ws/src" className="h-64" title="Source Browser" />
            </div>
          </div>
        </div>

        {/* Custom Label Examples */}
        <div className="bg-white rounded-lg shadow-lg p-6">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">Custom Label Examples</h2>
          <p className="text-gray-600 mb-6">
            Examples showing different ways to customize the display labels for target files.
          </p>
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
            <div>
              <h3 className="text-md font-medium text-gray-900 mb-2">Default Label</h3>
              <p className="text-sm text-gray-600 mb-3">
                Without <code>targetFileLabel</code> - shows "Contains filename"
              </p>
              <FileBrowser
                targetFileName="package.json"
                className="h-64"
                title="Node.js Projects"
              />
            </div>
            <div>
              <h3 className="text-md font-medium text-gray-900 mb-2">Emoji Label</h3>
              <p className="text-sm text-gray-600 mb-3">Using emojis for visual distinction</p>
              <FileBrowser
                targetFileName="package.json"
                targetFileLabel="📦 Node.js 프로젝트"
                className="h-64"
                title="Node.js Projects"
              />
            </div>
            <div>
              <h3 className="text-md font-medium text-gray-900 mb-2">Custom Text</h3>
              <p className="text-sm text-gray-600 mb-3">Using descriptive custom text</p>
              <FileBrowser
                targetFileName="package.json"
                targetFileLabel="Node.js project detected"
                className="h-64"
                title="Node.js Projects"
              />
            </div>
          </div>
        </div>
      </div>

      {/* Modals */}
      <FileBrowserModal
        isOpen={showModal === 'basic'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        title="Select Any File"
      />

      <FileBrowserModal
        isOpen={showModal === 'images'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        fileFilter={imageFilter}
        title="Select Image File"
        selectButtonText="Select Image"
      />

      <FileBrowserModal
        isOpen={showModal === 'config'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        fileFilter={configFilter}
        title="Select Configuration File"
        selectButtonText="Select Config"
      />

      <FileBrowserModal
        isOpen={showModal === 'directory'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        title="Select Directory"
        selectButtonText="Select Directory"
        allowDirectorySelect={true}
      />

      <FileBrowserModal
        isOpen={showModal === 'ros_package'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        targetFileName="package.xml"
        targetFileLabel="📦 ROS 패키지"
        title="Select ROS Package Directory"
        selectButtonText="Select Package"
        allowDirectorySelect={true}
      />

      <FileBrowserModal
        isOpen={showModal === 'cmake_project'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        targetFileName="CMakeLists.txt"
        targetFileLabel="🔨 빌드 프로젝트"
        title="Select CMake Project Directory"
        selectButtonText="Select Project"
        allowDirectorySelect={true}
      />

      <FileBrowserModal
        isOpen={showModal === 'workspace_home'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        homePath="/home/ola/ai_ws"
        title="Browse with Workspace as Home"
        selectButtonText="Select"
        allowDirectorySelect={true}
      />

      <FileBrowserModal
        isOpen={showModal === 'project_home'}
        onClose={() => setShowModal(false)}
        onFileSelect={handleModalFileSelect}
        homePath="/home/ola/ai_ws/src/physical_ai_tools"
        title="Browse with Project as Home"
        selectButtonText="Select"
        allowDirectorySelect={true}
      />
    </div>
  );
}
