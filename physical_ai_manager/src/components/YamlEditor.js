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
import yaml from 'js-yaml';
import clsx from 'clsx';

function YamlEditor({ onYamlLoad }) {
  const [fileContent, setFileContent] = useState(() => {
    // Load saved file content from local storage
    const savedContent = localStorage.getItem('yamlFileContent');
    return savedContent ? JSON.parse(savedContent) : null;
  });

  const [editableContent, setEditableContent] = useState(() => {
    // Load saved editing content from local storage
    const savedEditableContent = localStorage.getItem('yamlEditableContent');
    return savedEditableContent || '';
  });

  // Add state to save original YAML text
  const [originalYamlText, setOriginalYamlText] = useState(() => {
    // Load saved original YAML text from local storage
    return localStorage.getItem('yamlOriginalText') || '';
  });

  const [fileName, setFileName] = useState(() => {
    // Load saved file name from local storage
    return localStorage.getItem('yamlFileName') || '';
  });

  const [isEditing, setIsEditing] = useState(false);
  const [errorLine, setErrorLine] = useState(null);
  const [errorMessage, setErrorMessage] = useState('');

  // Save fileContent to local storage and notify parent component when it changes
  useEffect(() => {
    if (fileContent) {
      localStorage.setItem('yamlFileContent', JSON.stringify(fileContent));
      if (onYamlLoad) {
        onYamlLoad(fileContent);
      }
    }
  }, [fileContent, onYamlLoad]);

  // Save editableContent to local storage when it changes
  useEffect(() => {
    if (editableContent) {
      localStorage.setItem('yamlEditableContent', editableContent);
    }
  }, [editableContent]);

  // Save originalYamlText to local storage when it changes
  useEffect(() => {
    if (originalYamlText) {
      localStorage.setItem('yamlOriginalText', originalYamlText);
    }
  }, [originalYamlText]);

  // Save fileName to local storage when it changes
  useEffect(() => {
    if (fileName) {
      localStorage.setItem('yamlFileName', fileName);
    }
  }, [fileName]);

  // Sync editableContent and originalYamlText when component initializes
  useEffect(() => {
    if (editableContent && !originalYamlText) {
      setOriginalYamlText(editableContent);
    }
  }, []);

  const handleFileSelect = (event) => {
    const file = event.target.files[0];
    if (file) {
      setFileName(file.name);
      const reader = new FileReader();
      reader.onload = (e) => {
        try {
          // Save original text
          const yamlText = e.target.result;
          setOriginalYamlText(yamlText); // Save original YAML text

          // Parse using default method
          const content = yaml.load(yamlText);
          setFileContent(content);

          // Use original text for editing
          setEditableContent(yamlText);
          setIsEditing(false);
          setErrorLine(null);
          setErrorMessage('');
        } catch (error) {
          console.error('YAML parsing error:', error);
          alert(`YAML file parsing error: ${error.message}`);
        }
      };
      reader.readAsText(file);
    }
  };

  const handleApply = () => {
    try {
      // Only perform YAML syntax validation
      const parsedContent = yaml.load(editableContent);

      // Use current editing content without converting to an object
      // Instead, parse the current editing content to update fileContent state
      setFileContent(parsedContent);

      // Update original text when applying
      setOriginalYamlText(editableContent);

      // Notify parent component
      if (onYamlLoad) {
        onYamlLoad(parsedContent);
      }

      // End editing mode
      setIsEditing(false);
      setErrorLine(null);
      setErrorMessage('');
    } catch (error) {
      // Extract line number when YAML error occurs
      const lineMatch = error.message.match(/line (\d+)/);
      if (lineMatch && lineMatch[1]) {
        setErrorLine(parseInt(lineMatch[1]));
        setErrorMessage(error.message);
      }
      alert('Invalid YAML format');
    }
  };

  const handleDownload = () => {
    const blob = new Blob([editableContent], { type: 'text/yaml' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = fileName || 'config.yaml';
    document.body.appendChild(a);
    a.click();
    window.URL.revokeObjectURL(url);
    document.body.removeChild(a);
  };

  const handleCancel = () => {
    // Restore original content - use original YAML text
    setEditableContent(originalYamlText);
    setIsEditing(false);
    setErrorLine(null);
    setErrorMessage('');
  };

  const renderYamlWithLineNumbers = () => {
    if (!editableContent) return null;

    const lines = editableContent.split('\n');
    return (
      <div
        className={clsx(
          'absolute',
          'top-0',
          'left-0',
          'w-full',
          'h-full',
          'm-0',
          'p-4',
          'font-mono'
        )}
      >
        {lines.map((line, index) => (
          <div key={index} className={errorLine === index + 1 ? 'bg-red-200 block w-full' : ''}>
            {line}
          </div>
        ))}
        {errorLine && (
          <div className="sticky bottom-0 bg-red-100 p-2 border-t border-red-500 text-xs">
            Error (line {errorLine}): {errorMessage}
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="mt-10 w-full">
      <div className="mb-4 text-3xl font-semibold">Load Configuration File</div>
      <input type="file" accept=".yaml,.yml" onChange={handleFileSelect} className="text-base" />
      {fileContent && (
        <div className="mt-5 text-base">
          <div className="flex justify-between items-center mb-2.5">
            <h3>File Content:</h3>
            <div className="flex gap-2">
              {isEditing ? (
                <>
                  <button
                    onClick={handleCancel}
                    className="py-2 px-4 text-sm text-white border-none rounded cursor-pointer bg-gray-500"
                  >
                    Cancel
                  </button>
                  <button
                    onClick={handleApply}
                    className="py-2 px-4 text-sm text-white border-none rounded cursor-pointer bg-green-500"
                  >
                    Apply
                  </button>
                </>
              ) : (
                <>
                  <button
                    onClick={() => setIsEditing(true)}
                    className="py-2 px-4 text-sm text-white border-none rounded cursor-pointer bg-blue-500"
                  >
                    Edit
                  </button>
                  <button
                    onClick={handleDownload}
                    className="py-2 px-4 text-sm text-white border-none rounded cursor-pointer bg-orange-500"
                  >
                    Download
                  </button>
                </>
              )}
            </div>
          </div>
          <div className="w-full h-[500px] border border-gray-300 rounded-lg bg-gray-100 relative">
            {isEditing ? (
              <textarea
                value={editableContent}
                onChange={(e) => setEditableContent(e.target.value)}
                className={clsx(
                  'absolute',
                  'top-0',
                  'left-0',
                  'w-full',
                  'h-full',
                  'font-mono',
                  'text-sm',
                  'p-4',
                  'bg-transparent',
                  'border-none',
                  'outline-none',
                  'resize-none',
                  'box-border',
                  'whitespace-pre',
                  'leading-relaxed',
                  'overflow-auto'
                )}
              />
            ) : (
              renderYamlWithLineNumbers()
            )}
          </div>
        </div>
      )}
    </div>
  );
}

export default YamlEditor;
