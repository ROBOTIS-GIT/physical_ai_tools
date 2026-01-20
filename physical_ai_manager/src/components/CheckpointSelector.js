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
// Author: Claude (AI Assistant)

import React, { useState, useEffect, useCallback } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import {
  MdRefresh,
  MdCheckCircle,
  MdFolder,
  MdDelete,
  MdWarning,
  MdClose,
  MdMemory,
} from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { setSelectedCheckpoint } from '../features/training/trainingSlice';

export default function CheckpointSelector({ onSelect, disabled = false }) {
  const dispatch = useDispatch();

  const selectedCheckpoint = useSelector((state) => state.training.selectedCheckpoint);
  const isTraining = useSelector((state) => state.training.isTraining);

  const { callService } = useRosServiceCaller();

  const [checkpoints, setCheckpoints] = useState([]);
  const [loading, setLoading] = useState(false);
  const [deleteModalOpen, setDeleteModalOpen] = useState(false);
  const [checkpointToDelete, setCheckpointToDelete] = useState(null);
  const [deleting, setDeleting] = useState(false);

  // Fetch checkpoint list from service
  const fetchCheckpoints = useCallback(async () => {
    setLoading(true);
    try {
      const result = await callService(
        '/lerobot/checkpoint_list',
        'physical_ai_interfaces/srv/GetCheckpointList',
        {}
      );

      if (result && result.success && result.data) {
        setCheckpoints(result.data.checkpoints || []);
        toast.success(result.message || 'Checkpoints loaded successfully');
      } else {
        setCheckpoints([]);
        toast.error(result?.message || 'Failed to load checkpoints');
      }
    } catch (error) {
      console.error('Error fetching checkpoints:', error);
      setCheckpoints([]);
      toast.error(`Failed to load checkpoints: ${error.message}`);
    } finally {
      setLoading(false);
    }
  }, [callService]);

  // Handle checkpoint selection
  const handleCheckpointSelect = useCallback(
    (checkpoint) => {
      if (disabled || isTraining) return;

      dispatch(setSelectedCheckpoint(checkpoint));
      if (onSelect) {
        onSelect(checkpoint);
      }
      toast.success(`Selected: ${checkpoint.run_name}/${checkpoint.checkpoint_name}`);
    },
    [dispatch, onSelect, disabled, isTraining]
  );

  // Handle delete confirmation
  const handleDeleteClick = useCallback(
    (e, checkpoint) => {
      e.stopPropagation();
      if (disabled || isTraining) return;

      setCheckpointToDelete(checkpoint);
      setDeleteModalOpen(true);
    },
    [disabled, isTraining]
  );

  // Confirm delete
  const handleDeleteConfirm = useCallback(async () => {
    if (!checkpointToDelete) return;

    setDeleting(true);
    try {
      const result = await callService(
        '/lerobot/checkpoint_delete',
        'physical_ai_interfaces/srv/DeleteCheckpoint',
        { checkpoint_path: checkpointToDelete.path }
      );

      if (result && result.success) {
        toast.success('Checkpoint deleted successfully');
        // Clear selection if deleted checkpoint was selected
        if (selectedCheckpoint?.path === checkpointToDelete.path) {
          dispatch(setSelectedCheckpoint(null));
        }
        // Refresh the list
        fetchCheckpoints();
      } else {
        toast.error(result?.message || 'Failed to delete checkpoint');
      }
    } catch (error) {
      console.error('Error deleting checkpoint:', error);
      toast.error(`Failed to delete checkpoint: ${error.message}`);
    } finally {
      setDeleting(false);
      setDeleteModalOpen(false);
      setCheckpointToDelete(null);
    }
  }, [checkpointToDelete, callService, selectedCheckpoint, dispatch, fetchCheckpoints]);

  // Fetch checkpoints on mount
  useEffect(() => {
    fetchCheckpoints();
  }, [fetchCheckpoints]);

  // Format date for display
  const formatDate = (dateString) => {
    if (!dateString) return 'Unknown';
    try {
      const date = new Date(dateString);
      return date.toLocaleString('en-US', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
      });
    } catch {
      return dateString;
    }
  };

  // Format size for display
  const formatSize = (sizeMb) => {
    if (sizeMb === undefined || sizeMb === null) return 'Unknown';
    if (sizeMb < 1) return `${(sizeMb * 1024).toFixed(1)} KB`;
    if (sizeMb >= 1024) return `${(sizeMb / 1024).toFixed(2)} GB`;
    return `${sizeMb.toFixed(1)} MB`;
  };

  // Style classes following existing patterns
  const classContainer = clsx(
    'flex',
    'flex-col',
    'gap-3',
    'p-6',
    'bg-white',
    'rounded-2xl',
    'shadow-md',
    'border',
    'border-gray-200',
    'min-w-[560px]'
  );

  const classHeader = clsx(
    'flex',
    'items-center',
    'justify-between',
    'mb-2'
  );

  const classTitle = clsx('text-xl', 'font-bold', 'text-gray-700');

  const classRefreshButton = clsx(
    'flex',
    'items-center',
    'gap-2',
    'px-3',
    'py-2',
    'text-sm',
    'rounded-lg',
    'transition-colors',
    'bg-gray-50',
    'text-gray-700',
    'border',
    'border-gray-200',
    'hover:bg-gray-100',
    'disabled:opacity-50',
    'disabled:cursor-not-allowed'
  );

  const classListContainer = clsx(
    'border',
    'border-gray-200',
    'rounded-lg',
    'max-h-[400px]',
    'overflow-y-auto',
    'bg-gray-50'
  );

  const classCheckpointItem = (isSelected) =>
    clsx(
      'flex',
      'flex-col',
      'gap-2',
      'p-4',
      'cursor-pointer',
      'transition-all',
      'duration-150',
      'border-b',
      'border-gray-200',
      'last:border-b-0',
      {
        'bg-blue-50 border-l-4 border-l-blue-500': isSelected,
        'hover:bg-gray-100': !isSelected && !disabled,
        'opacity-60 cursor-not-allowed': disabled,
      }
    );

  const classCheckpointHeader = clsx(
    'flex',
    'items-center',
    'justify-between',
    'gap-3'
  );

  const classCheckpointTitle = (isSelected) =>
    clsx('flex', 'items-center', 'gap-2', 'font-medium', {
      'text-blue-700': isSelected,
      'text-gray-800': !isSelected,
    });

  const classCheckpointMeta = clsx(
    'flex',
    'flex-wrap',
    'items-center',
    'gap-x-4',
    'gap-y-1',
    'text-sm',
    'text-gray-600'
  );

  const classMetaItem = clsx('flex', 'items-center', 'gap-1');

  const classLatestBadge = clsx(
    'px-2',
    'py-0.5',
    'text-xs',
    'font-medium',
    'rounded-full',
    'bg-emerald-100',
    'text-emerald-700'
  );

  const classDeleteButton = clsx(
    'p-1.5',
    'rounded-md',
    'text-gray-400',
    'hover:text-red-600',
    'hover:bg-red-50',
    'transition-colors',
    'disabled:opacity-50',
    'disabled:cursor-not-allowed'
  );

  const classEmptyState = clsx(
    'flex',
    'flex-col',
    'items-center',
    'justify-center',
    'py-12',
    'text-gray-500'
  );

  const classLoadingState = clsx(
    'flex',
    'items-center',
    'justify-center',
    'py-12',
    'text-gray-500',
    'gap-2'
  );

  // Delete confirmation modal styles
  const classModalOverlay = clsx(
    'fixed',
    'inset-0',
    'bg-black/50',
    'flex',
    'items-center',
    'justify-center',
    'z-50'
  );

  const classModalContent = clsx(
    'bg-white',
    'rounded-2xl',
    'shadow-xl',
    'p-6',
    'max-w-md',
    'w-full',
    'mx-4'
  );

  const classModalHeader = clsx(
    'flex',
    'items-center',
    'gap-3',
    'mb-4'
  );

  const classModalTitle = clsx('text-lg', 'font-semibold', 'text-gray-800');

  const classModalBody = clsx('text-sm', 'text-gray-600', 'mb-6');

  const classModalActions = clsx('flex', 'justify-end', 'gap-3');

  const classCancelButton = clsx(
    'px-4',
    'py-2',
    'text-sm',
    'font-medium',
    'rounded-lg',
    'border',
    'border-gray-300',
    'text-gray-700',
    'hover:bg-gray-50',
    'transition-colors'
  );

  const classConfirmDeleteButton = clsx(
    'px-4',
    'py-2',
    'text-sm',
    'font-medium',
    'rounded-lg',
    'bg-red-600',
    'text-white',
    'hover:bg-red-700',
    'transition-colors',
    'disabled:opacity-50',
    'disabled:cursor-not-allowed'
  );

  return (
    <>
      <div className={classContainer}>
        <div className={classHeader}>
          <h3 className={classTitle}>Available Checkpoints</h3>
          <button
            onClick={fetchCheckpoints}
            disabled={loading || isTraining || disabled}
            className={classRefreshButton}
            title="Refresh checkpoint list"
          >
            <MdRefresh size={18} className={loading ? 'animate-spin' : ''} />
            <span>{loading ? 'Loading...' : 'Refresh'}</span>
          </button>
        </div>

        <div className={classListContainer}>
          {loading ? (
            <div className={classLoadingState}>
              <MdRefresh size={20} className="animate-spin" />
              <span>Loading checkpoints...</span>
            </div>
          ) : checkpoints.length === 0 ? (
            <div className={classEmptyState}>
              <MdFolder size={48} className="text-gray-300 mb-3" />
              <p className="font-medium">No checkpoints available</p>
              <p className="text-sm mt-1">Train a model to create checkpoints</p>
            </div>
          ) : (
            checkpoints.map((checkpoint, index) => {
              const isSelected = selectedCheckpoint?.path === checkpoint.path;
              const uniqueKey = checkpoint.path || `${checkpoint.run_name}-${checkpoint.checkpoint_name}-${index}`;

              return (
                <div
                  key={uniqueKey}
                  onClick={() => handleCheckpointSelect(checkpoint)}
                  className={classCheckpointItem(isSelected)}
                >
                  <div className={classCheckpointHeader}>
                    <div className={classCheckpointTitle(isSelected)}>
                      {isSelected ? (
                        <MdCheckCircle size={20} className="text-blue-600 flex-shrink-0" />
                      ) : (
                        <MdMemory size={20} className="text-gray-400 flex-shrink-0" />
                      )}
                      <span className="truncate">
                        {checkpoint.run_name} / {checkpoint.checkpoint_name === 'last' ? 'last' : `step ${checkpoint.step}`}
                      </span>
                      {checkpoint.is_latest && (
                        <span className={classLatestBadge}>Latest</span>
                      )}
                    </div>
                    <button
                      onClick={(e) => handleDeleteClick(e, checkpoint)}
                      disabled={disabled || isTraining}
                      className={classDeleteButton}
                      title="Delete checkpoint"
                    >
                      <MdDelete size={18} />
                    </button>
                  </div>
                  <div className={classCheckpointMeta}>
                    <span className={classMetaItem}>
                      <span className="text-gray-400">Policy:</span>
                      <span className="font-medium text-gray-700 uppercase">{checkpoint.policy_type}</span>
                    </span>
                    <span className={classMetaItem}>
                      <span className="text-gray-400">Dataset:</span>
                      <span className="font-medium text-gray-700">{checkpoint.dataset}</span>
                    </span>
                    <span className={classMetaItem}>
                      <span className="text-gray-400">Size:</span>
                      <span className="font-medium text-gray-700">{formatSize(checkpoint.size_mb)}</span>
                    </span>
                    <span className={classMetaItem}>
                      <span className="text-gray-400">Created:</span>
                      <span className="font-medium text-gray-700">{formatDate(checkpoint.created_at)}</span>
                    </span>
                  </div>
                </div>
              );
            })
          )}
        </div>

        <p className="text-xs text-gray-500 mt-1">
          Select a checkpoint for inference or to resume training
        </p>
      </div>

      {/* Delete Confirmation Modal */}
      {deleteModalOpen && (
        <div className={classModalOverlay} onClick={() => !deleting && setDeleteModalOpen(false)}>
          <div className={classModalContent} onClick={(e) => e.stopPropagation()}>
            <div className={classModalHeader}>
              <MdWarning size={24} className="text-red-500" />
              <h4 className={classModalTitle}>Delete Checkpoint</h4>
              <button
                onClick={() => !deleting && setDeleteModalOpen(false)}
                className="ml-auto p-1 text-gray-400 hover:text-gray-600"
                disabled={deleting}
              >
                <MdClose size={20} />
              </button>
            </div>
            <div className={classModalBody}>
              <p>Are you sure you want to delete this checkpoint?</p>
              {checkpointToDelete && (
                <p className="mt-2 font-mono text-sm bg-gray-100 p-2 rounded-lg truncate">
                  {checkpointToDelete.run_name}/{checkpointToDelete.checkpoint_name}
                </p>
              )}
              <p className="mt-3 text-red-600 font-medium">This action cannot be undone.</p>
            </div>
            <div className={classModalActions}>
              <button
                onClick={() => setDeleteModalOpen(false)}
                disabled={deleting}
                className={classCancelButton}
              >
                Cancel
              </button>
              <button
                onClick={handleDeleteConfirm}
                disabled={deleting}
                className={classConfirmDeleteButton}
              >
                {deleting ? 'Deleting...' : 'Delete'}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
