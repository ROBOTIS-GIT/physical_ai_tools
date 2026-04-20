// Copyright 2026 ROBOTIS CO., LTD.
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
// Author: Seongwoo Kim

import React, { useState, useCallback, useEffect, useMemo } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import {
  ReactFlow,
  Controls,
  Background,
  useNodesState,
  useEdgesState,
} from '@xyflow/react';
import '@xyflow/react/dist/style.css';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MdPlayArrow, MdStop, MdUploadFile } from 'react-icons/md';

import BTControlNode from '../components/bt/BTControlNode';
import BTActionNode from '../components/bt/BTActionNode';
import BTParamPanel from '../components/bt/BTParamPanel';
import FileBrowserModal from '../components/FileBrowserModal';
import { parseBTXml } from '../utils/btTreeParser';
import { setTreeXml, setTreeFileName, setBtStatus, setActiveNodeNames, setSelectedNodeId } from '../features/btmanager/btmanagerSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { DEFAULT_PATHS } from '../constants/paths';

const nodeTypes = {
  btControl: BTControlNode,
  btAction: BTActionNode,
};

export default function BTManagerPage({ isActive = true }) {
  const dispatch = useDispatch();
  const { callService } = useRosServiceCaller();
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);

  const treeXml = useSelector((state) => state.btmanager.treeXml);
  const treeFileName = useSelector((state) => state.btmanager.treeFileName);
  const btStatus = useSelector((state) => state.btmanager.btStatus);
  const activeNodeNames = useSelector((state) => state.btmanager.activeNodeNames);
  const selectedNodeId = useSelector((state) => state.btmanager.selectedNodeId);

  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [parseError, setParseError] = useState(null);
  const [showFileBrowser, setShowFileBrowser] = useState(false);

  // Parse XML and update flow whenever treeXml changes
  useEffect(() => {
    if (!treeXml) {
      setNodes([]);
      setEdges([]);
      setParseError(null);
      return;
    }

    try {
      const { nodes: newNodes, edges: newEdges } = parseBTXml(treeXml);
      setNodes(newNodes);
      setEdges(newEdges);
      setParseError(null);
    } catch (err) {
      setParseError(err.message);
      setNodes([]);
      setEdges([]);
    }
  }, [treeXml, setNodes, setEdges]);

  // Handle file selection from FileBrowserModal
  const handleServerFileSelect = useCallback(async (item) => {
    if (!item || !item.full_path) return;

    try {
      // On HTTPS the same-origin /api/ nginx proxy reaches the backend without
      // mixed-content issues; on HTTP keep the direct :8082 URL for backward compat.
      const useProxy =
        typeof window !== 'undefined' && window.location.protocol === 'https:';
      const fileUrl = useProxy
        ? `${window.location.origin}/api${item.full_path}`
        : (() => {
            const urlMatch = rosbridgeUrl.match(/wss?:\/\/([^:/]+)/);
            const host = urlMatch ? urlMatch[1] : 'localhost';
            return `http://${host}:8082${item.full_path}`;
          })();
      const response = await fetch(fileUrl);

      if (!response.ok) {
        throw new Error(`Failed to fetch file: ${response.status}`);
      }

      const xmlContent = await response.text();
      const fileName = item.name || item.full_path.split('/').pop();

      dispatch(setTreeXml(xmlContent));
      dispatch(setTreeFileName(fileName));
      toast.success(`Loaded: ${fileName}`);
    } catch (err) {
      toast.error(`Failed to load file: ${err.message}`);
    }
  }, [rosbridgeUrl, dispatch]);

  // Node click handler for param editing
  const handleNodeClick = useCallback((event, node) => {
    dispatch(setSelectedNodeId(node.id));
  }, [dispatch]);

  // Helper: get HTTP server base URL from rosbridgeUrl
  const getHttpBaseUrl = useCallback(() => {
    if (typeof window !== 'undefined' && window.location.protocol === 'https:') {
      return `${window.location.origin}/api`;
    }
    const urlMatch = rosbridgeUrl.match(/wss?:\/\/([^:/]+)/);
    const host = urlMatch ? urlMatch[1] : 'localhost';
    return `http://${host}:8082`;
  }, [rosbridgeUrl]);

  // BT Start - launch node, load XML, and run
  const handleStart = useCallback(async () => {
    if (!treeXml) {
      toast.error('No tree loaded');
      return;
    }
    try {
      // 1. Launch BT node if not running
      const baseUrl = getHttpBaseUrl();
      const launchRes = await fetch(`${baseUrl}/bt/launch`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({}),
      });
      const launchData = await launchRes.json();
      if (!launchData.success) {
        toast.error(`Failed to launch BT node: ${launchData.message}`);
        return;
      }

      // 2. Wait for BT node to fully initialize
      const isAlreadyRunning = launchData.message.includes('already running');
      if (!isAlreadyRunning) {
        await new Promise((resolve) => setTimeout(resolve, 8000));
      }

      // 3. Load XML and start execution
      const result = await callService(
        '/bt/load_and_run',
        'physical_ai_interfaces/srv/LoadAndRunTree',
        { tree_xml: treeXml },
        30000
      );
      if (result.success) {
        dispatch(setBtStatus('running'));
        dispatch(setSelectedNodeId(null));
        toast.success('BT started');
      } else {
        toast.error(`Failed: ${result.message}`);
      }
    } catch (err) {
      toast.error(`Failed to start BT: ${err.message}`);
    }
  }, [callService, dispatch, treeXml, getHttpBaseUrl]);

  // BT Stop - stop tree and shutdown node
  const handleStop = useCallback(async () => {
    try {
      // 1. Stop BT execution
      try {
        await callService(
          '/bt/set_running',
          'std_srvs/srv/SetBool',
          { data: false }
        );
      } catch {
        // BT node may already be gone, continue to shutdown
      }

      // 2. Shutdown BT node process
      const baseUrl = getHttpBaseUrl();
      await fetch(`${baseUrl}/bt/shutdown`, { method: 'POST' });

      dispatch(setBtStatus('stopped'));
      dispatch(setActiveNodeNames([]));
      toast.success('BT stopped');
    } catch (err) {
      toast.error(`Failed to stop BT: ${err.message}`);
    }
  }, [callService, dispatch, getHttpBaseUrl]);

  // Subscribe to BT status topic
  useEffect(() => {
    if (!rosbridgeUrl || !isActive) return;

    let ros = null;
    let statusTopic = null;
    let activeNodesTopic = null;

    const setupSubscription = async () => {
      try {
        const ROSLIB = (await import('roslib')).default;
        const { default: rosConnectionManager } = await import('../utils/rosConnectionManager');
        ros = await rosConnectionManager.getConnection(rosbridgeUrl);

        statusTopic = new ROSLIB.Topic({
          ros,
          name: '/bt/status',
          messageType: 'std_msgs/msg/String',
        });

        statusTopic.subscribe((msg) => {
          dispatch(setBtStatus(msg.data));
          if (msg.data !== 'running') {
            dispatch(setActiveNodeNames([]));
          }
        });

        activeNodesTopic = new ROSLIB.Topic({
          ros,
          name: '/bt/active_nodes',
          messageType: 'std_msgs/msg/String',
        });

        activeNodesTopic.subscribe((msg) => {
          const names = msg.data ? msg.data.split(',') : [];
          dispatch(setActiveNodeNames(names));
        });
      } catch (err) {
        console.debug('BT status subscription not available:', err.message);
      }
    };

    setupSubscription();

    return () => {
      if (statusTopic) statusTopic.unsubscribe();
      if (activeNodesTopic) activeNodesTopic.unsubscribe();
    };
  }, [rosbridgeUrl, isActive, dispatch]);

  // Annotate nodes with isActive flag based on active node IDs
  const annotatedNodes = useMemo(() => {
    const activeSet = new Set(activeNodeNames);
    return nodes.map((node) => ({
      ...node,
      data: {
        ...node.data,
        isActive: activeSet.has(node.id),
        isSelected: node.id === selectedNodeId,
      },
    }));
  }, [nodes, activeNodeNames, selectedNodeId]);

  const statusColor = btStatus === 'running' ? 'bg-green-500' : 'bg-gray-400';
  const statusLabel = btStatus === 'running' ? 'Running' : 'Stopped';

  return (
    <div className="w-full h-full flex flex-col">
      {/* Header */}
      <div className="flex items-center justify-between px-6 py-4 border-b border-gray-200 bg-white">
        <h1 className="text-xl font-bold text-gray-800">BT Manager</h1>
        <div className="flex items-center gap-3">
          <span className="text-sm text-gray-500">
            {treeFileName || 'No file loaded'}
          </span>
          <button
            onClick={() => setShowFileBrowser(true)}
            className={clsx(
              'flex items-center gap-2 px-4 py-2 rounded-lg cursor-pointer',
              'bg-gray-100 hover:bg-gray-200 text-gray-700 text-sm font-medium',
              'transition-colors duration-150'
            )}
          >
            <MdUploadFile size={18} />
            Load XML
          </button>
        </div>
      </div>

      {/* React Flow Canvas */}
      <div className="flex-1 relative flex">
        <div className="flex-1 relative">
        {parseError ? (
          <div className="flex items-center justify-center h-full">
            <div className="text-red-500 text-center">
              <p className="font-semibold">Parse Error</p>
              <p className="text-sm mt-1">{parseError}</p>
            </div>
          </div>
        ) : nodes.length === 0 ? (
          <div className="flex items-center justify-center h-full text-gray-400">
            <div className="text-center">
              <p className="text-lg">No behavior tree loaded</p>
              <p className="text-sm mt-1">Click "Load XML" to select a tree file</p>
            </div>
          </div>
        ) : (
          <ReactFlow
            nodes={annotatedNodes}
            edges={edges}
            onNodesChange={onNodesChange}
            onEdgesChange={onEdgesChange}
            nodeTypes={nodeTypes}
            fitView
            fitViewOptions={{ padding: 0.2 }}
            nodesDraggable={false}
            nodesConnectable={false}
            elementsSelectable={true}
            onNodeClick={handleNodeClick}
            minZoom={0.3}
            maxZoom={2}
            zoomOnScroll={false}
            panOnScroll={true}
            zoomOnPinch={true}
            zoomActivationKeyCode="Control"
          >
            <Controls showInteractive={false} />
            <Background color="#e5e7eb" gap={16} />
          </ReactFlow>
        )}
        </div>
        {selectedNodeId && (
          <BTParamPanel
            nodes={annotatedNodes}
            selectedNodeId={selectedNodeId}
          />
        )}
      </div>

      {/* Bottom Control Bar */}
      <div className="flex items-center justify-between px-6 py-3 border-t border-gray-200 bg-white">
        <div className="flex items-center gap-3">
          <button
            onClick={handleStart}
            disabled={btStatus === 'running' || !treeXml}
            className={clsx(
              'flex items-center gap-2 px-5 py-2 rounded-lg text-sm font-medium transition-colors',
              (btStatus === 'running' || !treeXml)
                ? 'bg-gray-200 text-gray-400 cursor-not-allowed'
                : 'bg-green-600 hover:bg-green-700 text-white'
            )}
          >
            <MdPlayArrow size={20} />
            Start
          </button>
          <button
            onClick={handleStop}
            disabled={btStatus === 'stopped'}
            className={clsx(
              'flex items-center gap-2 px-5 py-2 rounded-lg text-sm font-medium transition-colors',
              btStatus === 'stopped'
                ? 'bg-gray-200 text-gray-400 cursor-not-allowed'
                : 'bg-red-600 hover:bg-red-700 text-white'
            )}
          >
            <MdStop size={20} />
            Stop
          </button>
        </div>

        <div className="flex items-center gap-2">
          <div className={clsx('w-3 h-3 rounded-full', statusColor)} />
          <span className="text-sm text-gray-600">{statusLabel}</span>
        </div>
      </div>

      {/* File Browser Modal */}
      <FileBrowserModal
        isOpen={showFileBrowser}
        onClose={() => setShowFileBrowser(false)}
        onFileSelect={handleServerFileSelect}
        initialPath={DEFAULT_PATHS.BT_TREES_PATH}
        defaultPath={DEFAULT_PATHS.BT_TREES_PATH}
        title="Select BT XML File"
        selectButtonText="Load"
        fileFilter={(item) => item.name.endsWith('.xml')}
      />
    </div>
  );
}
