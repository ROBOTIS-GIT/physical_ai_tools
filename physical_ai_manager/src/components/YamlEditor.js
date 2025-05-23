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
import './YamlEditor.css';

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
      <div className="yaml-editor-preview">
        {lines.map((line, index) => (
          <div key={index} className={errorLine === index + 1 ? 'yaml-editor-error-line' : ''}>
            {line}
          </div>
        ))}
        {errorLine && (
          <div className="yaml-editor-error-message">
            Error (line {errorLine}): {errorMessage}
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="yaml-editor-container">
      <div className="yaml-editor-title">Load Configuration File</div>
      <input
        type="file"
        accept=".yaml,.yml"
        onChange={handleFileSelect}
        className="yaml-editor-file-input"
      />
      {fileContent && (
        <div className="yaml-editor-content">
          <div className="yaml-editor-header">
            <h3>File Content:</h3>
            <div className="yaml-editor-buttons">
              {isEditing ? (
                <>
                  <button
                    onClick={handleCancel}
                    className="yaml-editor-button yaml-editor-button-cancel"
                  >
                    Cancel
                  </button>
                  <button
                    onClick={handleApply}
                    className="yaml-editor-button yaml-editor-button-apply"
                  >
                    Apply
                  </button>
                </>
              ) : (
                <button
                  onClick={() => setIsEditing(true)}
                  className="yaml-editor-button yaml-editor-button-edit"
                >
                  Edit
                </button>
              )}
              <button
                onClick={handleDownload}
                className="yaml-editor-button yaml-editor-button-download"
              >
                Download
              </button>
            </div>
          </div>
          <div className="yaml-editor-textarea-container">
            {isEditing ? (
              <textarea
                value={editableContent}
                onChange={(e) => {
                  setEditableContent(e.target.value);
                  setErrorLine(null); // Reset error display when editing
                  setErrorMessage('');
                }}
                className="yaml-editor-textarea"
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
