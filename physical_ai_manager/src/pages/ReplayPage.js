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
// Author: Dongyun Kim

import React, { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { MdFolder, MdPlayArrow, MdPause, MdRefresh, MdUnfoldMore, MdUnfoldLess, MdReplay, MdKeyboardArrowUp, MdKeyboardArrowDown, MdClose } from 'react-icons/md';
import toast from 'react-hot-toast';
import clsx from 'clsx';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import FileBrowserModal from '../components/FileBrowserModal';
import { JointChart } from '../components/replay';
import {
  lttbDownsample,
  formatFileSize,
  formatDateTime,
  formatTime,
} from '../utils/chartUtils';
import { useKeyboardShortcuts } from '../hooks/useKeyboardShortcuts';
import {
  setSelectedBagPath,
  setLoading,
  setReplayData,
  setTaskMarkers,
  setError,
  setCurrentTime,
  setIsPlaying,
  setIsVideoLoaded,
  setVideoLoadProgress,
} from '../features/replay/replaySlice';

const STATE_COLOR = '#dc2626';
const ACTION_COLOR = '#2563eb';

function ReplayPage({ isActive }) {
  const dispatch = useDispatch();
  const { getReplayData, getRosbagList } = useRosServiceCaller();
  const rosHost = useSelector((state) => state.ros.rosHost);

  // Redux state
  const {
    selectedBagPath,
    isLoading,
    isLoaded,
    error,
    videoFiles,
    videoNames,
    videoFps,
    videoServerPort,
    bagPath,
    jointTimestamps,
    jointNames,
    jointPositions,
    actionTimestamps,
    actionNames,
    actionValues,
    duration,
    currentTime,
    isPlaying,
    isVideoLoaded,
    // eslint-disable-next-line no-unused-vars
    videoLoadProgress,
    // Extended metadata
    robotType,
    recordingDate,
    fileSizeBytes,
    taskMarkers,
    frameCounts,
  } = useSelector((state) => state.replay);

  // Local state
  const [showFileBrowser, setShowFileBrowser] = useState(false);
  const [expandedJoints, setExpandedJoints] = useState(new Set());
  const [videoBlobUrls, setVideoBlobUrls] = useState([]); // Downloaded video blob URLs
  const [downloadProgress, setDownloadProgress] = useState(0); // 0-100
  const [isDownloading, setIsDownloading] = useState(false);
  const [expandedVideoIndex, setExpandedVideoIndex] = useState(null); // For 2x video expansion
  const [rosbagList, setRosbagList] = useState([]); // List of ROSbags in parent folder
  const [currentBagIndex, setCurrentBagIndex] = useState(-1); // Current bag index in list
  const [parentFolderPath, setParentFolderPath] = useState(''); // Parent folder path
  const [playbackSpeed, setPlaybackSpeed] = useState(1); // Playback speed (1x, 1.5x, 2x, 3x)

  // A-B Loop state
  const [loopStart, setLoopStart] = useState(null); // A point (seconds)
  const [loopEnd, setLoopEnd] = useState(null); // B point (seconds)

  // Task Marker state
  const [showMarkerDialog, setShowMarkerDialog] = useState(false); // Marker add dialog
  const [pendingMarkerTime, setPendingMarkerTime] = useState(null); // Time for new marker
  const [markerInput, setMarkerInput] = useState(''); // Custom instruction input
  const [isSavingMarkers, setIsSavingMarkers] = useState(false); // Saving state
  const [instructionPalette, setInstructionPalette] = useState(() => {
    // Load from localStorage on initial render
    try {
      const saved = localStorage.getItem('taskInstructionPalette');
      if (saved) {
        const parsed = JSON.parse(saved);
        return parsed.instructions || [];
      }
    } catch {
      // Ignore parse errors
    }
    return [
      'Pick up the object',
      'Move to target position',
      'Place the object',
      'Open gripper',
      'Close gripper',
    ];
  });
  const [newPaletteInput, setNewPaletteInput] = useState(''); // New palette instruction input
  const [showHelpModal, setShowHelpModal] = useState(false); // Keyboard shortcuts help
  // Trim points for valid data range
  const [trimStart, setTrimStart] = useState(null); // { time, frame, instruction }
  const [trimEnd, setTrimEnd] = useState(null); // { time, frame }
  const [showTrimStartDialog, setShowTrimStartDialog] = useState(false);
  const [trimStartInstruction, setTrimStartInstruction] = useState('');
  // Exclude regions - sections to skip/remove from training data
  const [excludeRegions, setExcludeRegions] = useState([]); // [{ start: { time, frame }, end: { time, frame } }, ...]
  const [pendingExcludeStart, setPendingExcludeStart] = useState(null); // Temp storage while marking exclude region

  // Available playback speeds
  const PLAYBACK_SPEEDS = [0.5, 1, 1.5, 2, 3];

  // Refs for multiple videos
  const videoRefs = useRef([]);
  // Cache for downloaded video blob URLs (persists until component unmount)
  const videoCacheRef = useRef(new Map()); // Map<bagPath, blobUrls[]>
  // Cache for task markers per bag (session-only, cleared on SW close)
  const markersCacheRef = useRef(new Map()); // Map<bagPath, taskMarkers[]>

  // Calculate video URL for a given index (for downloading)
  const getVideoUrl = useCallback(
    (index) => {
      if (!bagPath || !videoFiles.length) return null;
      const videoFile = videoFiles[index];
      if (!videoFile) return null;
      return `http://${rosHost}:${videoServerPort}/video/${bagPath}/${videoFile}`;
    },
    [bagPath, videoFiles, rosHost, videoServerPort]
  );

  // Download all videos as blobs for smooth playback (with caching)
  const downloadVideos = useCallback(async () => {
    if (!videoFiles.length || !bagPath) return;

    // Check cache first
    const cachedUrls = videoCacheRef.current.get(bagPath);
    if (cachedUrls && cachedUrls.length === videoFiles.length) {
      // Use cached blob URLs
      setVideoBlobUrls(cachedUrls);
      dispatch(setIsVideoLoaded(true));
      dispatch(setVideoLoadProgress(100));
      toast.success('Videos loaded from cache');
      return;
    }

    setIsDownloading(true);
    setDownloadProgress(0);

    const totalVideos = videoFiles.length;
    const progressPerVideo = 100 / totalVideos;
    const newBlobUrls = [];

    try {
      for (let i = 0; i < totalVideos; i++) {
        const url = getVideoUrl(i);
        if (!url) continue;

        // Fetch with progress tracking
        const response = await fetch(url);
        if (!response.ok) {
          throw new Error(`Failed to download video ${i + 1}`);
        }

        const contentLength = response.headers.get('content-length');
        const total = parseInt(contentLength, 10) || 0;
        const reader = response.body.getReader();
        const chunks = [];
        let received = 0;

        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          chunks.push(value);
          received += value.length;

          // Update progress
          if (total > 0) {
            const videoProgress = (received / total) * progressPerVideo;
            setDownloadProgress(Math.round(i * progressPerVideo + videoProgress));
          }
        }

        // Create blob URL
        const blob = new Blob(chunks, { type: 'video/mp4' });
        const blobUrl = URL.createObjectURL(blob);
        newBlobUrls.push(blobUrl);

        setDownloadProgress(Math.round((i + 1) * progressPerVideo));
      }

      // Save to cache
      videoCacheRef.current.set(bagPath, newBlobUrls);

      setVideoBlobUrls(newBlobUrls);
      dispatch(setIsVideoLoaded(true));
      dispatch(setVideoLoadProgress(100));
      toast.success('Videos downloaded successfully');
    } catch (error) {
      console.error('Video download failed:', error);
      toast.error(`Failed to download videos: ${error.message}`);
      dispatch(setError(error.message));
    } finally {
      setIsDownloading(false);
    }
  }, [videoFiles, bagPath, getVideoUrl, dispatch]);

  // Start downloading videos when replay data is loaded
  useEffect(() => {
    if (isLoaded && videoFiles.length > 0 && videoBlobUrls.length === 0 && !isDownloading) {
      downloadVideos();
    }
  }, [isLoaded, videoFiles.length, videoBlobUrls.length, isDownloading, downloadVideos]);

  // Cleanup all cached blob URLs on unmount only
  useEffect(() => {
    const cache = videoCacheRef.current;
    return () => {
      // Revoke all cached blob URLs when component unmounts
      cache.forEach((urls) => {
        urls.forEach((url) => {
          if (url) URL.revokeObjectURL(url);
        });
      });
      cache.clear();
    };
  }, []);

  // Get all unique joint names from both state and action
  const allJointNames = useMemo(() => {
    const names = new Set([...jointNames, ...actionNames]);
    return Array.from(names);
  }, [jointNames, actionNames]);

  // Check if we have action data
  const hasActionData = actionTimestamps.length > 0 && actionNames.length > 0;

  // Prepare state chart data - memoized for performance
  // Uses LTTB downsampling for each joint to preserve visual shape
  const stateChartData = useMemo(() => {
    if (!jointTimestamps.length || !jointNames.length || !jointPositions.length) {
      return [];
    }

    const numJoints = jointNames.length;
    const targetPoints = 1000; // Increased for better fidelity

    // First, create full data array
    const fullData = jointTimestamps.map((time, i) => {
      const point = { time };
      const startIdx = i * numJoints;
      jointNames.forEach((name, j) => {
        point[`state_${name}`] = jointPositions[startIdx + j] || 0;
      });
      return point;
    });

    // If data is small enough, return as is
    if (fullData.length <= targetPoints) {
      return fullData;
    }

    // Apply LTTB for each joint and merge results
    // We'll use the first joint as reference for time sampling
    const firstJointKey = `state_${jointNames[0]}`;
    const sampledData = lttbDownsample(fullData, targetPoints, 'time', firstJointKey);

    return sampledData;
  }, [jointTimestamps, jointNames, jointPositions]);

  // Prepare action chart data - memoized for performance
  // Uses LTTB downsampling for each action to preserve visual shape
  const actionChartData = useMemo(() => {
    if (!actionTimestamps.length || !actionNames.length || !actionValues.length) {
      return [];
    }

    const numActions = actionNames.length;
    const targetPoints = 1000; // Increased for better fidelity

    // First, create full data array
    const fullData = actionTimestamps.map((time, i) => {
      const point = { time };
      const startIdx = i * numActions;
      actionNames.forEach((name, j) => {
        point[`action_${name}`] = actionValues[startIdx + j] || 0;
      });
      return point;
    });

    // If data is small enough, return as is
    if (fullData.length <= targetPoints) {
      return fullData;
    }

    // Apply LTTB using the first action as reference
    const firstActionKey = `action_${actionNames[0]}`;
    const sampledData = lttbDownsample(fullData, targetPoints, 'time', firstActionKey);

    return sampledData;
  }, [actionTimestamps, actionNames, actionValues]);

  // Toggle joint expansion by row (groups of 3)
  const toggleJoint = useCallback((jointName) => {
    const jointIndex = allJointNames.indexOf(jointName);
    if (jointIndex === -1) return;

    // Calculate row start index (groups of 3)
    const rowStart = Math.floor(jointIndex / 3) * 3;
    const rowJoints = allJointNames.slice(rowStart, rowStart + 3);

    setExpandedJoints((prev) => {
      const newSet = new Set(prev);
      const isRowExpanded = rowJoints.some((name) => prev.has(name));

      if (isRowExpanded) {
        // Collapse entire row
        rowJoints.forEach((name) => newSet.delete(name));
      } else {
        // Expand entire row
        rowJoints.forEach((name) => newSet.add(name));
      }
      return newSet;
    });
  }, [allJointNames]);

  // Expand all joints
  const expandAllJoints = useCallback(() => {
    setExpandedJoints(new Set(allJointNames));
  }, [allJointNames]);

  // Collapse all joints
  const collapseAllJoints = useCallback(() => {
    setExpandedJoints(new Set());
  }, []);

  // Handle bag selection
  const handleSelectBag = async (path) => {
    setShowFileBrowser(false);

    // Save current markers to cache before switching (if there are unsaved markers)
    if (bagPath && taskMarkers.length > 0) {
      markersCacheRef.current.set(bagPath, [...taskMarkers]);
    }

    dispatch(setSelectedBagPath(path));
    dispatch(setLoading(true));

    // Get parent folder path and load rosbag list
    const parentPath = path.substring(0, path.lastIndexOf('/'));
    if (parentPath && parentPath !== parentFolderPath) {
      setParentFolderPath(parentPath);
      try {
        const listResult = await getRosbagList(parentPath);
        if (listResult.success && listResult.rosbags) {
          setRosbagList(listResult.rosbags);
          // Find current bag index
          const idx = listResult.rosbags.findIndex((bag) => bag.path === path);
          setCurrentBagIndex(idx);
        }
      } catch (err) {
        console.error('Failed to load rosbag list:', err);
      }
    } else {
      // Update current index in existing list
      const idx = rosbagList.findIndex((bag) => bag.path === path);
      setCurrentBagIndex(idx);
    }

    try {
      const result = await getReplayData(path);
      if (result.success) {
        // Check if there are cached markers for this bag
        const cachedMarkers = markersCacheRef.current.get(path);
        if (cachedMarkers && cachedMarkers.length > 0) {
          // Use cached markers instead of server data
          result.task_markers = cachedMarkers;
          toast('Restored cached markers');
        }
        dispatch(setReplayData(result));
        // Load trim points from server if available
        if (result.trim_points) {
          const tp = result.trim_points;
          if (tp.start) {
            setTrimStart({
              time: tp.start.time,
              frame: tp.start.frame,
              instruction: tp.start.instruction || 'Start',
            });
          }
          if (tp.end) {
            setTrimEnd({
              time: tp.end.time,
              frame: tp.end.frame,
            });
          }
        }
        // Load exclude regions if available
        if (result.exclude_regions && result.exclude_regions.length > 0) {
          setExcludeRegions(result.exclude_regions);
        }
        toast.success('Replay data loaded successfully');
      } else {
        dispatch(setError(result.message));
        toast.error(`Failed to load replay data: ${result.message}`);
      }
    } catch (err) {
      dispatch(setError(err.message));
      toast.error(`Error loading replay data: ${err.message}`);
    }
  };

  // Navigate to previous/next rosbag
  const navigateRosbag = useCallback(
    async (direction) => {
      if (rosbagList.length === 0 || currentBagIndex === -1) return;
      if (isDownloading) return; // Prevent navigation while downloading

      const newIndex = direction === 'prev' ? currentBagIndex - 1 : currentBagIndex + 1;
      if (newIndex < 0 || newIndex >= rosbagList.length) return;

      // Save current markers to cache before switching
      if (bagPath && taskMarkers.length > 0) {
        markersCacheRef.current.set(bagPath, [...taskMarkers]);
      }

      const newBag = rosbagList[newIndex];
      setCurrentBagIndex(newIndex);
      dispatch(setSelectedBagPath(newBag.path));
      dispatch(setLoading(true));

      // Reset current video state (don't revoke - keep in cache)
      setVideoBlobUrls([]);
      setDownloadProgress(0);
      dispatch(setIsVideoLoaded(false));

      // Reset trim points and exclude regions when switching bags
      setTrimStart(null);
      setTrimEnd(null);
      setLoopStart(null);
      setLoopEnd(null);
      setExcludeRegions([]);
      setPendingExcludeStart(null);

      try {
        const result = await getReplayData(newBag.path);
        if (result.success) {
          // Check if there are cached markers for this bag
          const cachedMarkers = markersCacheRef.current.get(newBag.path);
          if (cachedMarkers && cachedMarkers.length > 0) {
            // Use cached markers instead of server data
            result.task_markers = cachedMarkers;
            toast('Restored cached markers');
          }
          dispatch(setReplayData(result));
          // Load trim points from new bag if saved
          if (result.trim_points) {
            const tp = result.trim_points;
            if (tp.start) {
              setTrimStart({
                time: tp.start.time,
                frame: tp.start.frame,
                instruction: tp.start.instruction || 'Start',
              });
            }
            if (tp.end) {
              setTrimEnd({
                time: tp.end.time,
                frame: tp.end.frame,
              });
            }
          }
          // Load exclude regions if available
          if (result.exclude_regions && result.exclude_regions.length > 0) {
            setExcludeRegions(result.exclude_regions);
          }
          toast.success(`Loaded: ${newBag.name}`);
        } else {
          dispatch(setError(result.message));
          toast.error(`Failed to load: ${result.message}`);
        }
      } catch (err) {
        dispatch(setError(err.message));
        toast.error(`Error: ${err.message}`);
      }
    },
    [rosbagList, currentBagIndex, isDownloading, dispatch, getReplayData, bagPath, taskMarkers]
  );



  // Handle video events (simplified - videos are pre-downloaded)
  useEffect(() => {
    if (!isVideoLoaded || videoBlobUrls.length === 0) return;

    // Copy ref value to use in cleanup
    const videos = videoRefs.current;

    const handleTimeUpdate = () => {
      // Get time from the first available video (handles expanded view where only one video exists)
      const video = expandedVideoIndex !== null ? videos[expandedVideoIndex] : videos.find(v => v);
      if (video) {
        dispatch(setCurrentTime(video.currentTime));
      }
    };

    const handleEnded = () => {
      dispatch(setIsPlaying(false));
      dispatch(setCurrentTime(duration)); // Set to end time
    };

    // Add listeners to all videos (or just the expanded one)
    videos.forEach((video) => {
      if (video) {
        video.addEventListener('timeupdate', handleTimeUpdate);
        video.addEventListener('ended', handleEnded);
      }
    });

    return () => {
      videos.forEach((video) => {
        if (video) {
          video.removeEventListener('timeupdate', handleTimeUpdate);
          video.removeEventListener('ended', handleEnded);
        }
      });
    };
  }, [isVideoLoaded, videoBlobUrls.length, duration, dispatch, expandedVideoIndex]);

  // A-B Loop: Jump to A when reaching B
  useEffect(() => {
    if (!isPlaying || loopStart === null || loopEnd === null) return;

    // Check if current time has passed loop end
    if (currentTime >= loopEnd) {
      videoRefs.current.forEach((v) => {
        if (v) v.currentTime = loopStart;
      });
      dispatch(setCurrentTime(loopStart));
    }
  }, [currentTime, isPlaying, loopStart, loopEnd, dispatch]);

  // Handle play/pause for all videos (simplified - no buffering needed)
  const togglePlayPause = useCallback(() => {
    if (isPlaying) {
      videoRefs.current.forEach((video) => {
        if (video) video.pause();
      });
      dispatch(setIsPlaying(false));
    } else {
      const firstVideo = videoRefs.current[0];

      // If video ended, restart from beginning
      if (firstVideo && firstVideo.ended) {
        videoRefs.current.forEach((video) => {
          if (video) video.currentTime = 0;
        });
        dispatch(setCurrentTime(0));
      } else {
        // Sync all videos to the same time before playing
        const targetTime = firstVideo?.currentTime || 0;
        videoRefs.current.forEach((video) => {
          if (video) video.currentTime = targetTime;
        });
      }

      // Play all videos
      videoRefs.current.forEach((video) => {
        if (video) video.play().catch(() => { });
      });
      dispatch(setIsPlaying(true));
    }
  }, [isPlaying, dispatch]);

  // Restart playback from beginning
  const restartPlayback = useCallback(() => {
    videoRefs.current.forEach((video) => {
      if (video) {
        video.currentTime = 0;
        video.play().catch(() => { });
      }
    });
    dispatch(setCurrentTime(0));
    dispatch(setIsPlaying(true));
  }, [dispatch]);

  // Step frame forward or backward (for ← / → keys)
  const stepFrame = useCallback(
    (direction) => {
      if (!isVideoLoaded) return;

      // Pause if playing
      if (isPlaying) {
        videoRefs.current.forEach((v) => v?.pause());
        dispatch(setIsPlaying(false));
      }

      const fps = videoFps[0] || 30;
      const frameTime = 1 / fps;
      const delta = direction === 'forward' ? frameTime : -frameTime;
      const newTime = Math.max(0, Math.min(duration, currentTime + delta));

      videoRefs.current.forEach((v) => {
        if (v) v.currentTime = newTime;
      });
      dispatch(setCurrentTime(newTime));
    },
    [isVideoLoaded, isPlaying, videoFps, duration, currentTime, dispatch]
  );

  // Seek relative (for Shift + ← / → keys, ±5 seconds)
  const seekRelative = useCallback(
    (seconds) => {
      if (!isVideoLoaded) return;

      const newTime = Math.max(0, Math.min(duration, currentTime + seconds));
      videoRefs.current.forEach((v) => {
        if (v) v.currentTime = newTime;
      });
      dispatch(setCurrentTime(newTime));
    },
    [isVideoLoaded, duration, currentTime, dispatch]
  );

  // Toggle A-B loop points (press 'a' to set A, then B, then clear)
  const toggleLoopPoint = useCallback(() => {
    if (!isVideoLoaded) return;

    if (loopStart === null) {
      // Set A point
      setLoopStart(currentTime);
      toast(`Loop A set at ${formatTime(currentTime)}`);
    } else if (loopEnd === null) {
      // Set B point (must be after A)
      if (currentTime > loopStart) {
        setLoopEnd(currentTime);
        toast(`Loop B set at ${formatTime(currentTime)}`);
      } else {
        // If current time is before A, update A point
        setLoopStart(currentTime);
        toast(`Loop A updated to ${formatTime(currentTime)}`);
      }
    } else {
      // Both points set, clear and start over with new A
      setLoopStart(currentTime);
      setLoopEnd(null);
      toast(`Loop A set at ${formatTime(currentTime)}`);
    }
  }, [isVideoLoaded, loopStart, loopEnd, currentTime]);

  // Clear A-B loop
  const clearLoop = useCallback(() => {
    setLoopStart(null);
    setLoopEnd(null);
    toast('Loop cleared');
  }, []);

  // ===== Task Marker Callbacks =====

  // Open marker dialog at current time
  const openMarkerDialog = useCallback(() => {
    if (!isVideoLoaded) return;

    // Require Start to be set before adding markers
    if (!trimStart) {
      toast.error('Please set Start point first (press S)');
      return;
    }

    // Pause video if playing
    if (isPlaying) {
      videoRefs.current.forEach((v) => v?.pause());
      dispatch(setIsPlaying(false));
    }

    setPendingMarkerTime(currentTime);
    setMarkerInput('');
    setShowMarkerDialog(true);
  }, [isVideoLoaded, isPlaying, currentTime, dispatch, trimStart]);

  // Add a marker with given instruction
  const addMarker = useCallback((instruction) => {
    if (pendingMarkerTime === null || !instruction.trim()) return;

    const fps = videoFps[0] || 30;
    const frame = Math.round(pendingMarkerTime * fps);

    const newMarker = {
      frame: frame,
      time: pendingMarkerTime,
      instruction: instruction.trim(),
    };

    // Add and sort by frame (frame-based ordering)
    const updatedMarkers = [...taskMarkers, newMarker].sort((a, b) => a.frame - b.frame);
    dispatch(setTaskMarkers(updatedMarkers));

    setShowMarkerDialog(false);
    setPendingMarkerTime(null);
    setMarkerInput('');
    toast.success(`Marker added at frame ${frame}: "${instruction.trim()}"`);
  }, [pendingMarkerTime, videoFps, taskMarkers, dispatch]);

  // Delete the nearest marker to current time
  const deleteNearestMarker = useCallback(() => {
    if (!taskMarkers.length) {
      toast('No markers to delete');
      return;
    }

    // Find nearest marker
    let nearestIndex = 0;
    let minDiff = Math.abs(taskMarkers[0].time - currentTime);

    taskMarkers.forEach((marker, index) => {
      const diff = Math.abs(marker.time - currentTime);
      if (diff < minDiff) {
        minDiff = diff;
        nearestIndex = index;
      }
    });

    // Only delete if within 2 seconds
    if (minDiff > 2) {
      toast('No marker within 2 seconds');
      return;
    }

    const deletedMarker = taskMarkers[nearestIndex];
    const updatedMarkers = taskMarkers.filter((_, i) => i !== nearestIndex);
    dispatch(setTaskMarkers(updatedMarkers));
    toast(`Deleted: "${deletedMarker.instruction}"`);
  }, [taskMarkers, currentTime, dispatch]);

  // Jump to marker by index (1-9)
  const jumpToMarker = useCallback((index) => {
    if (index < 0 || index >= taskMarkers.length) {
      toast(`No marker at position ${index + 1}`);
      return;
    }

    const marker = taskMarkers[index];
    videoRefs.current.forEach((v) => {
      if (v) v.currentTime = marker.time;
    });
    dispatch(setCurrentTime(marker.time));
    toast(`Jumped to: "${marker.instruction}"`);
  }, [taskMarkers, dispatch]);

  // Open trim start dialog (shows dialog to enter instruction)
  const openTrimStartDialog = useCallback(() => {
    if (!isVideoLoaded) return;
    setTrimStartInstruction(trimStart?.instruction || '');
    setShowTrimStartDialog(true);
  }, [isVideoLoaded, trimStart]);

  // Set trim start with instruction at current position
  const applyTrimStart = useCallback((instruction) => {
    const fps = videoFps[0] || 30;
    const frame = Math.round(currentTime * fps);
    setTrimStart({
      time: currentTime,
      frame,
      instruction: instruction.trim() || 'Start',
    });
    setShowTrimStartDialog(false);
    toast(`Start point set at ${formatTime(currentTime)} (frame ${frame})`);
  }, [currentTime, videoFps]);

  // Set trim end at current time
  const applyTrimEnd = useCallback(() => {
    if (!isVideoLoaded) return;
    const fps = videoFps[0] || 30;
    const frame = Math.round(currentTime * fps);
    setTrimEnd({
      time: currentTime,
      frame,
    });
    toast(`End point set at ${formatTime(currentTime)}`);
  }, [isVideoLoaded, currentTime, videoFps]);

  // Clear trim points
  const clearTrimPoints = useCallback(() => {
    setTrimStart(null);
    setTrimEnd(null);
    toast('Trim points cleared');
  }, []);

  // Toggle exclude region marking (press x twice: first for start, second for end)
  const toggleExcludeRegion = useCallback(() => {
    if (!isVideoLoaded) return;

    const fps = videoFps[0] || 30;
    const frame = Math.round(currentTime * fps);

    if (pendingExcludeStart === null) {
      // First press - set exclude start
      setPendingExcludeStart({ time: currentTime, frame });
      toast(`Exclude start at ${formatTime(currentTime)} - press X again to set end`);
    } else {
      // Second press - set exclude end and create region
      if (currentTime <= pendingExcludeStart.time) {
        toast.error('Exclude end must be after start');
        return;
      }
      const newRegion = {
        start: pendingExcludeStart,
        end: { time: currentTime, frame },
      };
      setExcludeRegions((prev) => [...prev, newRegion].sort((a, b) => a.start.time - b.start.time));
      setPendingExcludeStart(null);
      toast.success(`Excluded: ${formatTime(newRegion.start.time)} - ${formatTime(newRegion.end.time)}`);
    }
  }, [isVideoLoaded, currentTime, videoFps, pendingExcludeStart]);

  // Cancel pending exclude region
  const cancelExcludeRegion = useCallback(() => {
    if (pendingExcludeStart) {
      setPendingExcludeStart(null);
      toast('Exclude marking cancelled');
    }
  }, [pendingExcludeStart]);

  // Delete an exclude region by index
  const deleteExcludeRegion = useCallback((index) => {
    setExcludeRegions((prev) => prev.filter((_, i) => i !== index));
    toast('Exclude region deleted');
  }, []);

  // Save markers and trim points to server
  const saveMarkers = useCallback(async () => {
    if (!bagPath) {
      toast.error('No bag selected');
      return;
    }

    setIsSavingMarkers(true);
    try {
      // Build save data including trim points
      const saveData = {
        task_markers: taskMarkers,
      };

      // Add trim points if set
      if (trimStart || trimEnd) {
        saveData.trim_points = {
          start: trimStart ? {
            time: trimStart.time,
            frame: trimStart.frame,
            instruction: trimStart.instruction,
          } : null,
          end: trimEnd ? {
            time: trimEnd.time,
            frame: trimEnd.frame,
          } : null,
        };
      }

      // Add exclude regions if any
      if (excludeRegions.length > 0) {
        saveData.exclude_regions = excludeRegions.map((region) => ({
          start: { time: region.start.time, frame: region.start.frame },
          end: { time: region.end.time, frame: region.end.frame },
        }));
      }

      const response = await fetch(
        `http://${rosHost}:${videoServerPort}/task-markers/${bagPath}`,
        {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(saveData),
        }
      );

      const result = await response.json();
      if (result.success) {
        toast.success('Data saved to file');
      } else {
        toast.error(result.message || 'Failed to save');
      }
    } catch (error) {
      toast.error(`Save failed: ${error.message}`);
    } finally {
      setIsSavingMarkers(false);
    }
  }, [bagPath, rosHost, videoServerPort, taskMarkers, trimStart, trimEnd, excludeRegions]);

  // Save instruction palette to localStorage
  const savePaletteToStorage = useCallback((newPalette) => {
    setInstructionPalette(newPalette);
    localStorage.setItem('taskInstructionPalette', JSON.stringify({
      name: 'Custom Palette',
      instructions: newPalette,
    }));
  }, []);

  // Add instruction to palette
  const addToPalette = useCallback((instruction) => {
    if (!instruction.trim()) return;
    if (instructionPalette.includes(instruction.trim())) {
      toast('Instruction already in palette');
      return;
    }
    const newPalette = [...instructionPalette, instruction.trim()];
    savePaletteToStorage(newPalette);
    toast.success('Added to palette');
  }, [instructionPalette, savePaletteToStorage]);

  // Remove instruction from palette
  const removeFromPalette = useCallback((index) => {
    const newPalette = instructionPalette.filter((_, i) => i !== index);
    savePaletteToStorage(newPalette);
  }, [instructionPalette, savePaletteToStorage]);

  // Import palette from JSON file
  const importPaletteFromJson = useCallback((event) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const data = JSON.parse(e.target.result);
        if (data.instructions && Array.isArray(data.instructions)) {
          savePaletteToStorage(data.instructions);
          toast.success(`Imported ${data.instructions.length} instructions`);
        } else {
          toast.error('Invalid JSON format');
        }
      } catch {
        toast.error('Failed to parse JSON file');
      }
    };
    reader.readAsText(file);
    // Reset input for re-upload
    event.target.value = '';
  }, [savePaletteToStorage]);

  // Export palette to JSON file
  const exportPaletteToJson = useCallback(() => {
    if (!instructionPalette.length) {
      toast('No instructions to export');
      return;
    }

    const data = {
      name: 'Instruction Palette',
      instructions: instructionPalette,
      exportedAt: new Date().toISOString(),
    };

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `instruction_palette_${new Date().toISOString().slice(0, 10)}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    toast.success('Palette exported');
  }, [instructionPalette]);

  // Get current active task based on time (returns marker and index)
  const { currentActiveTask, currentActiveTaskIndex } = useMemo(() => {
    if (!taskMarkers.length) return { currentActiveTask: null, currentActiveTaskIndex: -1 };

    // Find the last marker before or at current time
    let activeMarker = null;
    let activeIndex = -1;
    for (let i = 0; i < taskMarkers.length; i++) {
      if (taskMarkers[i].time <= currentTime) {
        activeMarker = taskMarkers[i];
        activeIndex = i;
      } else {
        break;
      }
    }
    return { currentActiveTask: activeMarker, currentActiveTaskIndex: activeIndex };
  }, [taskMarkers, currentTime]);

  // Keyboard shortcuts via useKeyboardShortcuts hook
  useKeyboardShortcuts({
    isActive,
    handlers: {
      onNavigatePrev: () => navigateRosbag('prev'),
      onNavigateNext: () => navigateRosbag('next'),
      onStepBackward: () => stepFrame('backward'),
      onStepForward: () => stepFrame('forward'),
      onSeekRelative: seekRelative,
      onTogglePlayPause: togglePlayPause,
      onToggleLoopPoint: toggleLoopPoint,
      onClearLoop: clearLoop,
      onOpenMarkerDialog: openMarkerDialog,
      onDeleteNearestMarker: deleteNearestMarker,
      onJumpToMarker: jumpToMarker,
      onSetTrimStart: openTrimStartDialog,
      onSetTrimEnd: applyTrimEnd,
      onToggleExcludeRegion: toggleExcludeRegion,
      onCancelExclude: cancelExcludeRegion,
      onToggleHelp: () => setShowHelpModal((prev) => !prev),
      onCloseHelp: () => setShowHelpModal(false),
      onCloseTrimDialog: () => setShowTrimStartDialog(false),
      hasPendingExclude: pendingExcludeStart,
      showHelpModal,
      showTrimDialog: showTrimStartDialog,
    },
  });

  // Change playback speed
  const changePlaybackSpeed = useCallback((speed) => {
    setPlaybackSpeed(speed);
    videoRefs.current.forEach((video) => {
      if (video) video.playbackRate = speed;
    });
  }, []);

  // Apply playback speed when videos are loaded
  useEffect(() => {
    videoRefs.current.forEach((video) => {
      if (video) video.playbackRate = playbackSpeed;
    });
  }, [videoBlobUrls, playbackSpeed]);

  // Handle seek for all videos
  const handleSeek = useCallback((e) => {
    if (!isVideoLoaded) return;

    const rect = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const percentage = x / rect.width;
    const newTime = percentage * duration;

    // Sync all videos to the new time
    videoRefs.current.forEach((video) => {
      if (video) video.currentTime = newTime;
    });
    dispatch(setCurrentTime(newTime));
  }, [isVideoLoaded, duration, dispatch]);

  // Handle chart click to seek video to specific time
  const handleChartSeek = useCallback((time) => {
    if (!isVideoLoaded || typeof time !== 'number') return;

    const clampedTime = Math.max(0, Math.min(duration, time));

    // Sync all videos to the new time
    videoRefs.current.forEach((video) => {
      if (video) video.currentTime = clampedTime;
    });
    dispatch(setCurrentTime(clampedTime));
  }, [isVideoLoaded, duration, dispatch]);

  // Extract short name from video file path
  const getShortVideoName = (filePath) => {
    const fileName = filePath.split('/').pop();
    // Remove .mp4 extension and common prefixes
    return fileName
      .replace('.mp4', '')
      .replace('_compressed', '')
      .split('_')
      .slice(-3)
      .join('_');
  };

  // Reset state when leaving page
  useEffect(() => {
    if (!isActive) {
      videoRefs.current.forEach((video) => {
        if (video) video.pause();
      });
      dispatch(setIsPlaying(false));
    }
  }, [isActive, dispatch]);

  // Clean up blob URLs when bag changes
  useEffect(() => {
    // Reset blob URLs when selected bag changes
    setVideoBlobUrls([]);
    setDownloadProgress(0);
  }, [selectedBagPath]);

  return (
    <div className="flex flex-col h-full p-6 bg-gray-50">
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <h1 className="text-2xl font-bold text-gray-800">Replay Viewer</h1>
        <button
          onClick={() => setShowFileBrowser(true)}
          className="flex items-center gap-2 px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
        >
          <MdFolder size={20} />
          Select ROSbag
        </button>
      </div>

      {/* Main content */}
      <div className="flex-1 flex gap-4 min-h-0">
        {/* Left panel - Video and Joint data */}
        <div className="flex-1 flex flex-col gap-4 min-h-0">
          {/* Video panel */}
          <div className="flex-1 flex flex-col bg-white rounded-xl shadow-sm overflow-hidden min-h-0">
            {isLoaded && videoFiles.length > 0 ? (
              <>
                {/* Video container - grid layout for multiple videos */}
                <div className={clsx(
                  "flex-1 bg-gray-100 flex items-center justify-center min-h-0 relative",
                  expandedVideoIndex !== null ? "p-2" : "p-4"
                )}>
                  {expandedVideoIndex !== null ? (
                    /* Expanded single video view */
                    <div
                      className="relative bg-white rounded-lg overflow-hidden shadow cursor-pointer flex items-center justify-center"
                      style={{ width: '100%', height: '100%' }}
                      onClick={() => setExpandedVideoIndex(null)}
                      title="Click to collapse"
                    >
                      {/* Video label */}
                      <div className="absolute top-2 left-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-sm rounded font-medium">
                        {videoNames[expandedVideoIndex] || getShortVideoName(videoFiles[expandedVideoIndex])}
                      </div>
                      {/* Collapse hint */}
                      <div className="absolute top-2 right-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-xs rounded">
                        Click to collapse
                      </div>
                      <video
                        ref={(el) => (videoRefs.current[expandedVideoIndex] = el)}
                        src={videoBlobUrls[expandedVideoIndex] || ''}
                        className="max-w-full max-h-full object-contain"
                        style={{ width: 'auto', height: '100%' }}
                        playsInline
                        muted={expandedVideoIndex > 0}
                      />
                    </div>
                  ) : (
                    /* Grid view */
                    <div
                      className={clsx(
                        'grid gap-4 w-full h-full',
                        videoFiles.length === 1 && 'grid-cols-1',
                        videoFiles.length === 2 && 'grid-cols-2',
                        videoFiles.length >= 3 && 'grid-cols-2 lg:grid-cols-3'
                      )}
                    >
                      {videoFiles.map((file, index) => (
                        <div
                          key={index}
                          className="relative bg-white rounded-lg overflow-hidden shadow flex flex-col cursor-pointer hover:ring-2 hover:ring-blue-400 transition-all"
                          onClick={() => setExpandedVideoIndex(index)}
                          title="Click to expand"
                        >
                          {/* Video label - use videoNames from config, fallback to short name */}
                          <div className="absolute top-2 left-2 z-10 px-2 py-1 bg-black bg-opacity-60 text-white text-xs rounded font-medium">
                            {videoNames[index] || getShortVideoName(file)}
                          </div>
                          <video
                            ref={(el) => (videoRefs.current[index] = el)}
                            src={videoBlobUrls[index] || ''}
                            className="w-full h-full object-contain bg-gray-50"
                            playsInline
                            muted={index > 0} // Mute all except first video
                          />
                        </div>
                      ))}
                    </div>
                  )}
                  {isDownloading && (
                    <div className="absolute inset-0 bg-white bg-opacity-90 flex flex-col items-center justify-center z-20">
                      <MdRefresh className="animate-spin text-blue-500 mb-4" size={48} />
                      <div className="text-gray-700 mb-4">
                        Downloading videos... {downloadProgress}%
                      </div>
                      <div className="w-64 h-2 bg-gray-300 rounded-full overflow-hidden">
                        <div
                          className="h-full bg-blue-500 transition-all duration-300"
                          style={{ width: `${downloadProgress}%` }}
                        />
                      </div>
                      <div className="mt-2 text-xs text-gray-500">
                        {Math.floor(downloadProgress / (100 / videoFiles.length))} / {videoFiles.length} videos
                      </div>
                    </div>
                  )}
                </div>

                {/* Video controls */}
                <div className="p-4 bg-gray-100 border-t">
                  {/* Timeline */}
                  <div
                    className={clsx(
                      'h-2 bg-gray-300 rounded-full mb-4 cursor-pointer relative',
                      { 'opacity-50 cursor-not-allowed': !isVideoLoaded }
                    )}
                    onClick={isVideoLoaded ? handleSeek : undefined}
                  >
                    {/* Trim region highlight (valid range) */}
                    {(trimStart || trimEnd) && duration > 0 && (
                      <div
                        className="absolute h-full bg-blue-100 rounded-full"
                        style={{
                          left: `${((trimStart?.time || 0) / duration) * 100}%`,
                          width: `${(((trimEnd?.time || duration) - (trimStart?.time || 0)) / duration) * 100}%`,
                        }}
                      />
                    )}
                    {/* A-B Loop region highlight */}
                    {loopStart !== null && loopEnd !== null && duration > 0 && (
                      <div
                        className="absolute h-full bg-yellow-300 opacity-40 rounded-full"
                        style={{
                          left: `${(loopStart / duration) * 100}%`,
                          width: `${((loopEnd - loopStart) / duration) * 100}%`,
                        }}
                      />
                    )}
                    {/* Exclude regions (grayed out with stripes) */}
                    {excludeRegions.map((region, idx) => (
                      <div
                        key={`exclude-${idx}`}
                        className="absolute h-full bg-red-400 opacity-50 rounded-full"
                        style={{
                          left: `${(region.start.time / duration) * 100}%`,
                          width: `${((region.end.time - region.start.time) / duration) * 100}%`,
                          backgroundImage: 'repeating-linear-gradient(45deg, transparent, transparent 2px, rgba(0,0,0,0.1) 2px, rgba(0,0,0,0.1) 4px)',
                        }}
                        title={`Excluded: ${formatTime(region.start.time)} - ${formatTime(region.end.time)}`}
                      />
                    ))}
                    {/* Pending exclude start marker */}
                    {pendingExcludeStart && duration > 0 && (
                      <div
                        className="absolute top-1/2 -translate-y-1/2 w-1 h-5 bg-red-500 z-20 animate-pulse"
                        style={{ left: `${(pendingExcludeStart.time / duration) * 100}%` }}
                        title={`Exclude start: ${formatTime(pendingExcludeStart.time)} - press X to set end`}
                      />
                    )}
                    {/* Progress bar */}
                    <div
                      className="h-full bg-blue-500 rounded-full relative z-10"
                      style={{ width: `${(currentTime / duration) * 100}%` }}
                    />
                    {/* A marker */}
                    {loopStart !== null && duration > 0 && (
                      <div
                        className="absolute top-1/2 -translate-y-1/2 w-1 h-4 bg-green-500 z-20"
                        style={{ left: `${(loopStart / duration) * 100}%` }}
                        title={`A: ${formatTime(loopStart)}`}
                      />
                    )}
                    {/* B marker */}
                    {loopEnd !== null && duration > 0 && (
                      <div
                        className="absolute top-1/2 -translate-y-1/2 w-1 h-4 bg-red-500 z-20"
                        style={{ left: `${(loopEnd / duration) * 100}%` }}
                        title={`B: ${formatTime(loopEnd)}`}
                      />
                    )}
                    {/* Task Markers */}
                    {taskMarkers.map((marker, idx) => (
                      <div
                        key={`marker-${idx}`}
                        className="absolute top-1/2 -translate-y-1/2 z-15 cursor-pointer group"
                        style={{ left: `${(marker.time / duration) * 100}%` }}
                        onClick={(e) => {
                          e.stopPropagation();
                          videoRefs.current.forEach((v) => {
                            if (v) v.currentTime = marker.time;
                          });
                          dispatch(setCurrentTime(marker.time));
                        }}
                        title={`#${idx + 1}: ${marker.instruction}`}
                      >
                        {/* Marker line */}
                        <div className="w-0.5 h-5 bg-purple-600 group-hover:bg-purple-400" />
                        {/* Marker number */}
                        <div className="absolute -top-4 left-1/2 -translate-x-1/2 text-xs font-bold text-purple-600 bg-white px-1 rounded shadow-sm opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap">
                          {idx + 1}
                        </div>
                      </div>
                    ))}
                    {/* Trim Start marker */}
                    {trimStart && duration > 0 && (
                      <div
                        className="absolute top-1/2 -translate-y-1/2 z-18 cursor-pointer group"
                        style={{ left: `${(trimStart.time / duration) * 100}%` }}
                        onClick={(e) => {
                          e.stopPropagation();
                          videoRefs.current.forEach((v) => {
                            if (v) v.currentTime = trimStart.time;
                          });
                          dispatch(setCurrentTime(trimStart.time));
                        }}
                        title={`Start: ${formatTime(trimStart.time)} - ${trimStart.instruction}`}
                      >
                        {/* Triangle marker pointing right */}
                        <div className="w-0 h-0 border-t-[6px] border-t-transparent border-b-[6px] border-b-transparent border-l-[8px] border-l-green-600 group-hover:border-l-green-400" />
                        {/* Label */}
                        <div className="absolute -top-5 left-0 text-xs font-bold text-green-600 bg-white px-1 rounded shadow-sm opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap">
                          S
                        </div>
                      </div>
                    )}
                    {/* Trim End marker */}
                    {trimEnd && duration > 0 && (
                      <div
                        className="absolute top-1/2 -translate-y-1/2 z-18 cursor-pointer group"
                        style={{ left: `${(trimEnd.time / duration) * 100}%` }}
                        onClick={(e) => {
                          e.stopPropagation();
                          videoRefs.current.forEach((v) => {
                            if (v) v.currentTime = trimEnd.time;
                          });
                          dispatch(setCurrentTime(trimEnd.time));
                        }}
                        title={`End: ${formatTime(trimEnd.time)}`}
                      >
                        {/* Triangle marker pointing left */}
                        <div className="w-0 h-0 border-t-[6px] border-t-transparent border-b-[6px] border-b-transparent border-r-[8px] border-r-red-600 group-hover:border-r-red-400" />
                        {/* Label */}
                        <div className="absolute -top-5 right-0 text-xs font-bold text-red-600 bg-white px-1 rounded shadow-sm opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap">
                          E
                        </div>
                      </div>
                    )}
                    {/* Playhead */}
                    <div
                      className="absolute top-1/2 -translate-y-1/2 w-4 h-4 bg-blue-600 rounded-full shadow z-30"
                      style={{ left: `calc(${(currentTime / duration) * 100}% - 8px)` }}
                    />
                  </div>

                  {/* Controls */}
                  <div className="flex items-center gap-4">
                    {/* Restart button */}
                    <button
                      onClick={restartPlayback}
                      disabled={!isVideoLoaded}
                      className={clsx(
                        'p-2 rounded-full transition-colors',
                        isVideoLoaded
                          ? 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                          : 'bg-gray-100 text-gray-400 cursor-not-allowed'
                      )}
                      title="Restart from beginning"
                    >
                      <MdReplay size={20} />
                    </button>
                    {/* Play/Pause button */}
                    <button
                      onClick={togglePlayPause}
                      disabled={!isVideoLoaded}
                      className={clsx(
                        'p-2 rounded-full transition-colors',
                        isVideoLoaded
                          ? 'bg-blue-600 text-white hover:bg-blue-700'
                          : 'bg-gray-300 text-gray-500 cursor-not-allowed'
                      )}
                    >
                      {isPlaying ? <MdPause size={24} /> : <MdPlayArrow size={24} />}
                    </button>
                    <span className="text-sm text-gray-600">
                      {formatTime(currentTime)} / {formatTime(duration)}
                    </span>
                    <span className="text-sm text-gray-400">
                      ({videoFiles.length} camera{videoFiles.length > 1 ? 's' : ''})
                    </span>
                    {/* A-B Loop status */}
                    {(loopStart !== null || loopEnd !== null) && (
                      <div className="flex items-center gap-2 px-2 py-1 bg-yellow-100 rounded text-xs">
                        <span className="text-green-600 font-medium">
                          A: {loopStart !== null ? formatTime(loopStart) : '--'}
                        </span>
                        <span className="text-gray-400">|</span>
                        <span className="text-red-600 font-medium">
                          B: {loopEnd !== null ? formatTime(loopEnd) : '--'}
                        </span>
                        <button
                          onClick={clearLoop}
                          className="ml-1 text-gray-500 hover:text-gray-700"
                          title="Clear loop (Backspace)"
                        >
                          <MdClose size={14} />
                        </button>
                      </div>
                    )}
                    {/* Current active task - show from task markers or trimStart */}
                    {/* Click to loop the task segment */}
                    {(currentActiveTask || trimStart) && (
                      <button
                        className="flex items-center gap-2 px-2 py-1 bg-purple-100 hover:bg-purple-200 rounded text-xs max-w-xs cursor-pointer transition-colors"
                        onClick={() => {
                          // Determine start and end of current task segment
                          let segmentStart, segmentEnd;
                          if (currentActiveTask) {
                            segmentStart = currentActiveTask.time;
                            // Find next task marker to determine end
                            const nextMarkerIdx = currentActiveTaskIndex + 1;
                            segmentEnd = nextMarkerIdx < taskMarkers.length
                              ? taskMarkers[nextMarkerIdx].time
                              : (trimEnd?.time || duration);
                          } else if (trimStart) {
                            segmentStart = trimStart.time;
                            segmentEnd = taskMarkers.length > 0
                              ? taskMarkers[0].time
                              : (trimEnd?.time || duration);
                          }
                          if (segmentStart !== undefined && segmentEnd !== undefined) {
                            setLoopStart(segmentStart);
                            setLoopEnd(segmentEnd);
                            // Seek to segment start and start playback
                            videoRefs.current.forEach((v) => {
                              if (v) {
                                v.currentTime = segmentStart;
                                v.play().catch(() => { });
                              }
                            });
                            dispatch(setCurrentTime(segmentStart));
                            dispatch(setIsPlaying(true));
                            toast(`Loop: ${formatTime(segmentStart)} - ${formatTime(segmentEnd)}`);
                          }
                        }}
                        title="Click to loop this task segment"
                      >
                        <span className="text-purple-600 font-bold">Task:</span>
                        <span className="text-purple-800 truncate" title={currentActiveTask?.instruction || trimStart?.instruction}>
                          {currentActiveTask?.instruction || trimStart?.instruction}
                        </span>
                      </button>
                    )}
                    {/* Trim point buttons */}
                    <div className="flex items-center gap-1">
                      <button
                        onClick={openTrimStartDialog}
                        disabled={!isVideoLoaded}
                        className={clsx(
                          'px-2 py-1 text-xs rounded transition-colors flex items-center gap-1',
                          trimStart
                            ? 'bg-green-500 text-white'
                            : isVideoLoaded
                              ? 'bg-green-100 text-green-700 hover:bg-green-200'
                              : 'bg-gray-100 text-gray-400 cursor-not-allowed'
                        )}
                        title={trimStart ? `Start: ${formatTime(trimStart.time)} - ${trimStart.instruction}` : 'Set Start Point (S)'}
                      >
                        S{trimStart && <span className="opacity-80">{formatTime(trimStart.time)}</span>}
                      </button>
                      <button
                        onClick={applyTrimEnd}
                        disabled={!isVideoLoaded}
                        className={clsx(
                          'px-2 py-1 text-xs rounded transition-colors flex items-center gap-1',
                          trimEnd
                            ? 'bg-red-500 text-white'
                            : isVideoLoaded
                              ? 'bg-red-100 text-red-700 hover:bg-red-200'
                              : 'bg-gray-100 text-gray-400 cursor-not-allowed'
                        )}
                        title={trimEnd ? `End: ${formatTime(trimEnd.time)}` : 'Set End Point (E)'}
                      >
                        E{trimEnd && <span className="opacity-80">{formatTime(trimEnd.time)}</span>}
                      </button>
                      {(trimStart || trimEnd) && (
                        <button
                          onClick={clearTrimPoints}
                          className="text-gray-400 hover:text-red-500 px-1"
                          title="Clear trim points"
                        >
                          <MdClose size={14} />
                        </button>
                      )}
                    </div>
                    {/* Add marker button */}
                    <button
                      onClick={openMarkerDialog}
                      disabled={!isVideoLoaded}
                      className={clsx(
                        'px-2 py-1 text-xs rounded transition-colors',
                        isVideoLoaded
                          ? 'bg-purple-100 text-purple-700 hover:bg-purple-200'
                          : 'bg-gray-100 text-gray-400 cursor-not-allowed'
                      )}
                      title="Add marker (m)"
                    >
                      + Marker
                    </button>
                    {/* Playback speed selector */}
                    <div className="flex items-center gap-1 ml-auto">
                      {PLAYBACK_SPEEDS.map((speed) => (
                        <button
                          key={speed}
                          onClick={() => changePlaybackSpeed(speed)}
                          disabled={!isVideoLoaded}
                          className={clsx(
                            'px-2 py-1 text-xs rounded transition-colors',
                            playbackSpeed === speed
                              ? 'bg-blue-600 text-white'
                              : isVideoLoaded
                                ? 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                                : 'bg-gray-100 text-gray-400 cursor-not-allowed'
                          )}
                        >
                          {speed}x
                        </button>
                      ))}
                      {/* Help button */}
                      <button
                        onClick={() => setShowHelpModal(true)}
                        className="ml-2 w-6 h-6 flex items-center justify-center rounded-full bg-gray-200 text-gray-600 hover:bg-gray-300 text-sm font-bold"
                        title="Keyboard shortcuts (?)"
                      >
                        ?
                      </button>
                    </div>
                  </div>
                </div>
              </>
            ) : (
              <div className="flex-1 flex items-center justify-center text-gray-500 bg-gray-50">
                {isLoading ? (
                  <div className="flex flex-col items-center gap-4">
                    <MdRefresh className="animate-spin" size={48} />
                    <span>Loading replay data...</span>
                  </div>
                ) : error ? (
                  <div className="text-center text-red-500">
                    <p className="font-semibold">Error loading data</p>
                    <p className="text-sm">{error}</p>
                  </div>
                ) : (
                  <div className="text-center">
                    <MdFolder size={64} className="mx-auto mb-4 text-gray-400" />
                    <p>Select a ROSbag to start viewing</p>
                  </div>
                )}
              </div>
            )}
          </div>

          {/* Joint data chart panel - hidden when video is expanded */}
          <div className={clsx(
            "bg-white rounded-xl shadow-sm p-4 flex flex-col transition-all duration-200",
            expandedVideoIndex !== null ? "h-0 overflow-hidden opacity-0 p-0" : "h-96"
          )}>
            {/* Header with legend and expand/collapse all buttons */}
            <div className="flex items-center justify-between mb-3">
              <div className="flex items-center gap-4">
                <h2 className="text-lg font-semibold text-gray-800">Joint Data</h2>
                {/* Legend */}
                <div className="flex items-center gap-3 text-xs">
                  <div className="flex items-center gap-1">
                    <div className="w-3 h-1 rounded" style={{ backgroundColor: STATE_COLOR }} />
                    <span className="text-gray-600">State</span>
                  </div>
                  {hasActionData && (
                    <div className="flex items-center gap-1">
                      <div className="w-3 h-1 rounded" style={{ backgroundColor: ACTION_COLOR }} />
                      <span className="text-gray-600">Action</span>
                    </div>
                  )}
                </div>
              </div>
              {allJointNames.length > 0 && (
                <div className="flex items-center gap-2">
                  <span className="text-xs text-gray-500">
                    {expandedJoints.size} / {allJointNames.length} expanded
                  </span>
                  <button
                    onClick={expandAllJoints}
                    className="flex items-center gap-1 px-2 py-1 text-xs text-gray-600 hover:bg-gray-100 rounded transition-colors"
                    title="Expand all"
                  >
                    <MdUnfoldMore size={16} />
                    All
                  </button>
                  <button
                    onClick={collapseAllJoints}
                    className="flex items-center gap-1 px-2 py-1 text-xs text-gray-600 hover:bg-gray-100 rounded transition-colors"
                    title="Collapse all"
                  >
                    <MdUnfoldLess size={16} />
                    None
                  </button>
                </div>
              )}
            </div>

            {/* Joint charts grid */}
            {(stateChartData.length > 0 || actionChartData.length > 0) && allJointNames.length > 0 ? (
              <div className="flex-1 overflow-y-auto">
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-2">
                  {allJointNames.map((name) => (
                    <JointChart
                      key={name}
                      name={name}
                      stateData={stateChartData}
                      actionData={actionChartData}
                      currentTime={currentTime}
                      duration={duration}
                      isExpanded={expandedJoints.has(name)}
                      onToggle={() => toggleJoint(name)}
                      hasAction={hasActionData && actionNames.includes(name)}
                      onSeek={handleChartSeek}
                    />
                  ))}
                </div>
              </div>
            ) : (
              <div className="flex-1 flex items-center justify-center text-gray-400">
                No joint data available
              </div>
            )}
          </div>
        </div>

        {/* Right panel - Metadata & ROSbag list sidebar */}
        {isLoaded && (
          <div className="w-64 flex flex-col gap-3 overflow-y-auto max-h-full">
            {/* Metadata panel */}
            <div className="bg-white rounded-xl shadow-sm overflow-hidden">
              <div className="p-3 border-b bg-gray-50">
                <h3 className="text-sm font-semibold text-gray-700">Metadata</h3>
              </div>
              <div className="p-3 space-y-2 text-sm">
                {/* Recording date */}
                <div className="flex justify-between">
                  <span className="text-gray-500">Recorded</span>
                  <span className="text-gray-700 font-medium text-right">
                    {formatDateTime(recordingDate)}
                  </span>
                </div>
                {/* Robot type */}
                {robotType && (
                  <div className="flex justify-between">
                    <span className="text-gray-500">Robot</span>
                    <span className="text-gray-700 font-medium">{robotType}</span>
                  </div>
                )}
                {/* File size */}
                <div className="flex justify-between">
                  <span className="text-gray-500">Size</span>
                  <span className="text-gray-700 font-medium">
                    {formatFileSize(fileSizeBytes)}
                  </span>
                </div>
                {/* Duration */}
                <div className="flex justify-between">
                  <span className="text-gray-500">Duration</span>
                  <span className="text-gray-700 font-medium">
                    {formatTime(duration)}
                  </span>
                </div>
                {/* Frame counts per camera */}
                {frameCounts && Object.keys(frameCounts).length > 0 && (
                  <div className="pt-2 border-t mt-2">
                    <div className="text-gray-500 mb-1">Frames</div>
                    {Object.entries(frameCounts).map(([camera, count]) => (
                      <div key={camera} className="flex justify-between text-xs pl-2">
                        <span className="text-gray-400 truncate">{camera}</span>
                        <span className="text-gray-600">{count}</span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </div>

            {/* Task Markers panel */}
            <div className="bg-white rounded-xl shadow-sm overflow-hidden">
              <div className="p-3 border-b bg-gray-50">
                <div className="flex items-center justify-between">
                  <h3 className="text-sm font-semibold text-gray-700">Task Markers</h3>
                  <span className="text-xs text-gray-500">{taskMarkers.length} markers</span>
                </div>
              </div>
              <div className="p-2 max-h-48 overflow-y-auto">
                {(trimStart || taskMarkers.length > 0) ? (
                  <div className="space-y-1">
                    {/* Start point as first marker (if set) */}
                    {trimStart && (
                      <div
                        className={clsx(
                          'flex items-center gap-2 p-1.5 rounded cursor-pointer group transition-colors',
                          !currentActiveTask && currentTime >= trimStart.time && (taskMarkers.length === 0 || currentTime < taskMarkers[0].time)
                            ? 'bg-green-100 border-l-2 border-green-500'
                            : 'hover:bg-gray-50'
                        )}
                        onClick={() => {
                          // Set A-B loop from Start to first marker (or End)
                          const segmentStart = trimStart.time;
                          const segmentEnd = taskMarkers.length > 0
                            ? taskMarkers[0].time
                            : (trimEnd?.time || duration);
                          setLoopStart(segmentStart);
                          setLoopEnd(segmentEnd);

                          // Seek to segment start and start playback
                          videoRefs.current.forEach((v) => {
                            if (v) {
                              v.currentTime = segmentStart;
                              v.play().catch(() => { });
                            }
                          });
                          dispatch(setCurrentTime(segmentStart));
                          dispatch(setIsPlaying(true));
                          toast(`Loop: ${formatTime(segmentStart)} - ${formatTime(segmentEnd)}`);
                        }}
                        title="Click to loop from Start"
                      >
                        <span className="w-5 h-5 flex items-center justify-center text-xs font-bold rounded bg-green-500 text-white">
                          S
                        </span>
                        <span className="text-xs w-12 text-green-700 font-medium">
                          {formatTime(trimStart.time)}
                        </span>
                        <span className="flex-1 text-xs truncate text-green-800" title={trimStart.instruction}>
                          {trimStart.instruction}
                        </span>
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            setTrimStart(null);
                            toast('Start point cleared');
                          }}
                          className="opacity-0 group-hover:opacity-100 text-gray-400 hover:text-red-500 transition-opacity"
                          title="Clear Start point"
                        >
                          <MdClose size={14} />
                        </button>
                      </div>
                    )}
                    {/* Task markers */}
                    {taskMarkers.map((marker, idx) => (
                      <div
                        key={`marker-item-${idx}`}
                        className={clsx(
                          'flex items-center gap-2 p-1.5 rounded cursor-pointer group transition-colors',
                          idx === currentActiveTaskIndex
                            ? 'bg-purple-100 border-l-2 border-purple-500'
                            : 'hover:bg-gray-50'
                        )}
                        onClick={() => {
                          // Set A-B loop for this task segment
                          const segmentStart = marker.time;
                          const nextMarkerIdx = idx + 1;
                          const segmentEnd = nextMarkerIdx < taskMarkers.length
                            ? taskMarkers[nextMarkerIdx].time
                            : (trimEnd?.time || duration);
                          setLoopStart(segmentStart);
                          setLoopEnd(segmentEnd);

                          // Seek to segment start and start playback
                          videoRefs.current.forEach((v) => {
                            if (v) {
                              v.currentTime = segmentStart;
                              v.play().catch(() => { });
                            }
                          });
                          dispatch(setCurrentTime(segmentStart));
                          dispatch(setIsPlaying(true));
                          toast(`Loop: ${formatTime(segmentStart)} - ${formatTime(segmentEnd)}`);
                        }}
                        title="Click to loop this task segment"
                      >
                        <span className={clsx(
                          'w-5 h-5 flex items-center justify-center text-xs font-bold rounded',
                          idx === currentActiveTaskIndex
                            ? 'bg-purple-500 text-white'
                            : 'bg-purple-100 text-purple-700'
                        )}>
                          {idx + 1}
                        </span>
                        <span className={clsx(
                          'text-xs w-12',
                          idx === currentActiveTaskIndex ? 'text-purple-700 font-medium' : 'text-gray-500'
                        )}>
                          {formatTime(marker.time)}
                        </span>
                        <span className={clsx(
                          'flex-1 text-xs truncate',
                          idx === currentActiveTaskIndex ? 'text-purple-900 font-medium' : 'text-gray-700'
                        )} title={marker.instruction}>
                          {marker.instruction}
                        </span>
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            const updated = taskMarkers.filter((_, i) => i !== idx);
                            dispatch(setTaskMarkers(updated));
                          }}
                          className="opacity-0 group-hover:opacity-100 text-gray-400 hover:text-red-500 transition-opacity"
                          title="Delete marker"
                        >
                          <MdClose size={14} />
                        </button>
                      </div>
                    ))}
                  </div>
                ) : (
                  <div className="text-xs text-gray-400 text-center py-2">
                    Press 'S' for Start, 'm' to add marker
                  </div>
                )}
              </div>
              {/* Trim points info (compact) */}
              {(trimStart || trimEnd) && (
                <div className="px-2 py-1.5 bg-blue-50 border-t text-xs">
                  <div className="flex justify-between items-center text-blue-700">
                    <span className="font-medium">Valid Range:</span>
                    <span>
                      {formatTime(trimStart?.time || 0)} → {formatTime(trimEnd?.time || duration)}
                      {trimStart?.instruction && (
                        <span className="ml-1 text-blue-500">({trimStart.instruction})</span>
                      )}
                    </span>
                  </div>
                </div>
              )}
              {/* Exclude Regions panel */}
              {(excludeRegions.length > 0 || pendingExcludeStart) && (
                <div className="px-2 py-1.5 bg-red-50 border-t text-xs">
                  <div className="flex justify-between items-center text-red-700 mb-1">
                    <span className="font-medium">Exclude Regions:</span>
                    <span>{excludeRegions.length} region{excludeRegions.length !== 1 ? 's' : ''}</span>
                  </div>
                  {pendingExcludeStart && (
                    <div className="text-red-500 text-xs italic mb-1 animate-pulse">
                      Marking exclude from {formatTime(pendingExcludeStart.time)}... (X to set end, Esc to cancel)
                    </div>
                  )}
                  {excludeRegions.length > 0 && (
                    <div className="space-y-0.5 max-h-20 overflow-y-auto">
                      {excludeRegions.map((region, idx) => (
                        <div key={`exclude-item-${idx}`} className="flex items-center justify-between text-red-600 group">
                          <span>
                            {formatTime(region.start.time)} → {formatTime(region.end.time)}
                          </span>
                          <button
                            onClick={() => deleteExcludeRegion(idx)}
                            className="opacity-0 group-hover:opacity-100 text-red-400 hover:text-red-600 transition-opacity"
                            title="Delete region"
                          >
                            <MdClose size={12} />
                          </button>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              )}
              {/* Save button - show if markers or trim points or exclude regions exist */}
              {(taskMarkers.length > 0 || trimStart || trimEnd || excludeRegions.length > 0) && (
                <div className="p-2 border-t bg-gray-50">
                  <button
                    onClick={saveMarkers}
                    disabled={isSavingMarkers}
                    className={clsx(
                      'w-full py-1.5 text-xs rounded transition-colors',
                      isSavingMarkers
                        ? 'bg-gray-200 text-gray-400 cursor-wait'
                        : 'bg-purple-600 text-white hover:bg-purple-700'
                    )}
                  >
                    {isSavingMarkers ? 'Saving...' : 'Save to File'}
                  </button>
                </div>
              )}
            </div>

            {/* Instruction Palette panel */}
            <div className="bg-white rounded-xl shadow-sm overflow-hidden">
              <div className="p-3 border-b bg-gray-50">
                <div className="flex items-center justify-between">
                  <h3 className="text-sm font-semibold text-gray-700">Instruction Palette</h3>
                  <div className="flex items-center gap-2">
                    <label className="text-xs text-blue-600 hover:text-blue-800 cursor-pointer">
                      <input
                        type="file"
                        accept=".json"
                        onChange={importPaletteFromJson}
                        className="hidden"
                      />
                      Import
                    </label>
                    <span className="text-gray-300">|</span>
                    <button
                      onClick={exportPaletteToJson}
                      className="text-xs text-blue-600 hover:text-blue-800"
                    >
                      Export
                    </button>
                  </div>
                </div>
              </div>
              <div className="p-2 max-h-32 overflow-y-auto">
                {instructionPalette.length > 0 ? (
                  <div className="space-y-1">
                    {instructionPalette.map((instruction, idx) => (
                      <div
                        key={`palette-${idx}`}
                        className="flex items-center gap-2 p-1 hover:bg-gray-50 rounded group"
                      >
                        <span className="w-4 text-xs text-gray-400 font-medium">
                          {idx < 9 ? idx + 1 : ''}
                        </span>
                        <span className="flex-1 text-xs text-gray-600 truncate" title={instruction}>
                          {instruction}
                        </span>
                        <button
                          onClick={() => removeFromPalette(idx)}
                          className="opacity-0 group-hover:opacity-100 text-gray-400 hover:text-red-500 transition-opacity"
                          title="Remove from palette"
                        >
                          <MdClose size={12} />
                        </button>
                      </div>
                    ))}
                  </div>
                ) : (
                  <div className="text-xs text-gray-400 text-center py-2">
                    No instructions in palette
                  </div>
                )}
              </div>
              {/* Add new instruction */}
              <div className="p-2 border-t bg-gray-50">
                <div className="flex gap-1">
                  <input
                    type="text"
                    value={newPaletteInput}
                    onChange={(e) => setNewPaletteInput(e.target.value)}
                    onKeyDown={(e) => {
                      if (e.key === 'Enter' && newPaletteInput.trim()) {
                        e.preventDefault();
                        addToPalette(newPaletteInput);
                        setNewPaletteInput('');
                      }
                    }}
                    placeholder="New instruction..."
                    className="flex-1 px-2 py-1 text-xs border rounded focus:outline-none focus:ring-1 focus:ring-blue-500"
                  />
                  <button
                    onClick={() => {
                      if (newPaletteInput.trim()) {
                        addToPalette(newPaletteInput);
                        setNewPaletteInput('');
                      }
                    }}
                    disabled={!newPaletteInput.trim()}
                    className={clsx(
                      'px-2 py-1 text-xs rounded transition-colors',
                      newPaletteInput.trim()
                        ? 'bg-blue-600 text-white hover:bg-blue-700'
                        : 'bg-gray-200 text-gray-400 cursor-not-allowed'
                    )}
                  >
                    Add
                  </button>
                </div>
              </div>
            </div>

            {/* ROSbag list panel */}
            {rosbagList.length > 1 && (
              <div className="flex-1 flex flex-col bg-white rounded-xl shadow-sm overflow-hidden">
                {/* Header with navigation buttons */}
                <div className="p-3 border-b bg-gray-50">
                  <div className="flex items-center justify-between mb-2">
                    <h3 className="text-sm font-semibold text-gray-700">ROSbag List</h3>
                    <span className="text-xs text-gray-500">
                      {currentBagIndex + 1} / {rosbagList.length}
                    </span>
                  </div>
                  <div className="flex items-center gap-2">
                    <button
                      onClick={() => navigateRosbag('prev')}
                      disabled={currentBagIndex <= 0 || isDownloading}
                      className={clsx(
                        'flex-1 flex items-center justify-center gap-1 px-3 py-1.5 rounded text-sm transition-colors',
                        currentBagIndex <= 0 || isDownloading
                          ? 'bg-gray-100 text-gray-400 cursor-not-allowed'
                          : 'bg-blue-100 text-blue-700 hover:bg-blue-200'
                      )}
                    >
                      <MdKeyboardArrowUp size={18} />
                      Prev
                    </button>
                    <button
                      onClick={() => navigateRosbag('next')}
                      disabled={currentBagIndex >= rosbagList.length - 1 || isDownloading}
                      className={clsx(
                        'flex-1 flex items-center justify-center gap-1 px-3 py-1.5 rounded text-sm transition-colors',
                        currentBagIndex >= rosbagList.length - 1 || isDownloading
                          ? 'bg-gray-100 text-gray-400 cursor-not-allowed'
                          : 'bg-blue-100 text-blue-700 hover:bg-blue-200'
                      )}
                    >
                      Next
                      <MdKeyboardArrowDown size={18} />
                    </button>
                  </div>
                  <p className="text-xs text-gray-400 mt-2 text-center">
                    Use ↑/↓ arrow keys to navigate
                  </p>
                </div>

                {/* ROSbag list */}
                <div className="flex-1 overflow-y-auto">
                  {rosbagList.map((bag, index) => (
                    <button
                      key={bag.path}
                      onClick={() => handleSelectBag(bag.path)}
                      disabled={isDownloading}
                      className={clsx(
                        'w-full text-left px-3 py-2 border-b border-gray-100 transition-colors',
                        index === currentBagIndex
                          ? 'bg-blue-50 border-l-4 border-l-blue-500'
                          : 'hover:bg-gray-50',
                        isDownloading && 'cursor-not-allowed opacity-50'
                      )}
                    >
                      <div className="text-sm font-medium text-gray-700 truncate">
                        {bag.name}
                      </div>
                      {bag.has_videos && (
                        <div className="text-xs text-green-600">Has videos</div>
                      )}
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>
        )}
      </div>

      {/* Bag info */}
      {selectedBagPath && (
        <div className="mt-2 text-sm text-gray-500">
          <span className="font-medium">Selected:</span> {selectedBagPath}
        </div>
      )}

      {/* File browser modal */}
      {showFileBrowser && (
        <FileBrowserModal
          isOpen={showFileBrowser}
          onClose={() => setShowFileBrowser(false)}
          onFileSelect={(item) => handleSelectBag(item.full_path)}
          title="Select ROSbag Directory"
          allowDirectorySelect={true}
          allowFileSelect={false}
          targetFileName="metadata.yaml"
          targetFolderName="videos"
          defaultPath="/workspace"
        />
      )}

      {/* Task Marker add dialog */}
      {showMarkerDialog && (
        <div
          className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50"
          onClick={() => setShowMarkerDialog(false)}
        >
          <div
            className="bg-white rounded-xl shadow-xl w-96 max-w-full mx-4"
            onClick={(e) => e.stopPropagation()}
            onKeyDown={(e) => {
              if (e.key === 'Escape') {
                setShowMarkerDialog(false);
              } else if (e.key === 'Enter' && markerInput.trim()) {
                addMarker(markerInput);
              } else if (e.key >= '1' && e.key <= '9') {
                const idx = parseInt(e.key) - 1;
                if (idx < instructionPalette.length) {
                  addMarker(instructionPalette[idx]);
                }
              }
            }}
          >
            {/* Dialog header */}
            <div className="p-4 border-b bg-gray-50 rounded-t-xl">
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold text-gray-800">Add Task Marker</h3>
                <button
                  onClick={() => setShowMarkerDialog(false)}
                  className="text-gray-400 hover:text-gray-600"
                >
                  <MdClose size={20} />
                </button>
              </div>
              <p className="text-sm text-gray-500 mt-1">
                Time: {formatTime(pendingMarkerTime || 0)}
              </p>
            </div>

            {/* Dialog body */}
            <div className="p-4">
              {/* Quick select from palette */}
              {instructionPalette.length > 0 && (
                <div className="mb-4">
                  <p className="text-xs text-gray-500 mb-2">Quick select (press 1-9):</p>
                  <div className="space-y-1 max-h-40 overflow-y-auto">
                    {instructionPalette.slice(0, 9).map((instruction, idx) => (
                      <button
                        key={`quick-${idx}`}
                        onClick={() => addMarker(instruction)}
                        className="w-full flex items-center gap-2 p-2 text-left hover:bg-purple-50 rounded transition-colors"
                      >
                        <span className="w-5 h-5 flex items-center justify-center bg-purple-100 text-purple-700 text-xs font-bold rounded">
                          {idx + 1}
                        </span>
                        <span className="text-sm text-gray-700 truncate">{instruction}</span>
                      </button>
                    ))}
                  </div>
                </div>
              )}

              {/* Custom input */}
              <div>
                <p className="text-xs text-gray-500 mb-2">Or enter custom instruction:</p>
                <div className="flex gap-2">
                  <input
                    type="text"
                    value={markerInput}
                    onChange={(e) => setMarkerInput(e.target.value)}
                    onKeyDown={(e) => {
                      if (e.key === 'Enter' && markerInput.trim()) {
                        e.preventDefault();
                        addMarker(markerInput);
                      }
                    }}
                    placeholder="Type instruction..."
                    className="flex-1 px-3 py-2 border rounded-lg text-sm focus:outline-none focus:ring-2 focus:ring-purple-500"
                    autoFocus
                  />
                  <button
                    onClick={() => addMarker(markerInput)}
                    disabled={!markerInput.trim()}
                    className={clsx(
                      'px-4 py-2 rounded-lg text-sm font-medium transition-colors',
                      markerInput.trim()
                        ? 'bg-purple-600 text-white hover:bg-purple-700'
                        : 'bg-gray-200 text-gray-400 cursor-not-allowed'
                    )}
                  >
                    Add
                  </button>
                </div>
                {/* Add to palette option */}
                {markerInput.trim() && !instructionPalette.includes(markerInput.trim()) && (
                  <button
                    onClick={() => addToPalette(markerInput)}
                    className="mt-2 text-xs text-blue-600 hover:text-blue-800"
                  >
                    + Add to palette
                  </button>
                )}
              </div>
            </div>

            {/* Dialog footer */}
            <div className="px-4 py-3 border-t bg-gray-50 rounded-b-xl text-xs text-gray-400">
              Press Escape to cancel
            </div>
          </div>
        </div>
      )}

      {/* Keyboard shortcuts help modal */}
      {showHelpModal && (
        <div
          className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50"
          onClick={() => setShowHelpModal(false)}
        >
          <div
            className="bg-white rounded-xl shadow-xl w-[500px] max-w-full mx-4 max-h-[80vh] overflow-hidden"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Header */}
            <div className="p-4 border-b bg-gray-50 rounded-t-xl">
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold text-gray-800">Keyboard Shortcuts</h3>
                <button
                  onClick={() => setShowHelpModal(false)}
                  className="text-gray-400 hover:text-gray-600"
                >
                  <MdClose size={20} />
                </button>
              </div>
            </div>

            {/* Body */}
            <div className="p-4 overflow-y-auto max-h-[60vh]">
              <div className="space-y-4">
                {/* Playback */}
                <div>
                  <h4 className="text-sm font-semibold text-gray-700 mb-2">Playback</h4>
                  <div className="space-y-1 text-sm">
                    <div className="flex justify-between"><span className="text-gray-600">Play / Pause</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">Space</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">1 Frame Backward</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">←</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">1 Frame Forward</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">→</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">5 Seconds Back</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">Shift + ←</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">5 Seconds Forward</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">Shift + →</kbd></div>
                  </div>
                </div>

                {/* A-B Loop */}
                <div>
                  <h4 className="text-sm font-semibold text-gray-700 mb-2">A-B Loop</h4>
                  <div className="space-y-1 text-sm">
                    <div className="flex justify-between"><span className="text-gray-600">Set A/B Point</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">A</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Clear Loop</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">Backspace</kbd></div>
                  </div>
                </div>

                {/* Task Markers */}
                <div>
                  <h4 className="text-sm font-semibold text-gray-700 mb-2">Task Markers</h4>
                  <div className="space-y-1 text-sm">
                    <div className="flex justify-between"><span className="text-gray-600">Add Marker</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">M</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Delete Nearest Marker</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">D</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Jump to Marker #N</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">1-9</kbd></div>
                  </div>
                </div>

                {/* Navigation */}
                <div>
                  <h4 className="text-sm font-semibold text-gray-700 mb-2">Navigation</h4>
                  <div className="space-y-1 text-sm">
                    <div className="flex justify-between"><span className="text-gray-600">Previous ROSbag</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">↑</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Next ROSbag</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">↓</kbd></div>
                  </div>
                </div>

                {/* Trim Points */}
                <div>
                  <h4 className="text-sm font-semibold text-gray-700 mb-2">Trim Points</h4>
                  <div className="space-y-1 text-sm">
                    <div className="flex justify-between"><span className="text-gray-600">Set Start Point</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">S</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Set End Point</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">E</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Mark Exclude Region</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">X</kbd><span className="text-xs text-gray-400">(2x)</span></div>
                  </div>
                </div>

                {/* Other */}
                <div>
                  <h4 className="text-sm font-semibold text-gray-700 mb-2">Other</h4>
                  <div className="space-y-1 text-sm">
                    <div className="flex justify-between"><span className="text-gray-600">Show This Help</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">?</kbd></div>
                    <div className="flex justify-between"><span className="text-gray-600">Close Dialog</span><kbd className="px-2 py-0.5 bg-gray-100 rounded text-xs">Esc</kbd></div>
                  </div>
                </div>
              </div>
            </div>

            {/* Footer */}
            <div className="px-4 py-3 border-t bg-gray-50 rounded-b-xl text-xs text-gray-400 text-center">
              Press Escape or click outside to close
            </div>
          </div>
        </div>
      )}

      {/* Trim Start dialog - for entering instruction */}
      {showTrimStartDialog && (
        <div
          className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50"
          onClick={() => setShowTrimStartDialog(false)}
        >
          <div
            className="bg-white rounded-xl shadow-xl w-96 max-w-full mx-4"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Dialog header */}
            <div className="p-4 border-b bg-green-50 rounded-t-xl">
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold text-gray-800">Set Start Point</h3>
                <button
                  onClick={() => setShowTrimStartDialog(false)}
                  className="text-gray-400 hover:text-gray-600"
                >
                  <MdClose size={20} />
                </button>
              </div>
              <p className="text-sm text-gray-500 mt-1">
                Time: {formatTime(currentTime)} (Frame: {Math.round(currentTime * (videoFps[0] || 30))})
              </p>
            </div>

            {/* Dialog body */}
            <div className="p-4">
              {/* Quick select from palette */}
              {instructionPalette.length > 0 && (
                <div className="mb-4">
                  <p className="text-xs text-gray-500 mb-2">Quick select from palette:</p>
                  <div className="space-y-1 max-h-40 overflow-y-auto">
                    {instructionPalette.slice(0, 9).map((instruction, idx) => (
                      <button
                        key={`trim-quick-${idx}`}
                        onClick={() => applyTrimStart(instruction)}
                        className="w-full flex items-center gap-2 p-2 text-left hover:bg-green-50 rounded transition-colors"
                      >
                        <span className="w-5 h-5 flex items-center justify-center bg-green-100 text-green-700 text-xs font-bold rounded">
                          {idx + 1}
                        </span>
                        <span className="text-sm text-gray-700 truncate">{instruction}</span>
                      </button>
                    ))}
                  </div>
                </div>
              )}

              {/* Custom input */}
              <div>
                <p className="text-xs text-gray-500 mb-2">Or enter instruction for this segment:</p>
                <div className="flex gap-2">
                  <input
                    type="text"
                    value={trimStartInstruction}
                    onChange={(e) => setTrimStartInstruction(e.target.value)}
                    onKeyDown={(e) => {
                      if (e.key === 'Enter') {
                        e.preventDefault();
                        applyTrimStart(trimStartInstruction);
                      }
                    }}
                    placeholder="e.g., Pick up the cube"
                    className="flex-1 px-3 py-2 border rounded-lg text-sm focus:outline-none focus:ring-2 focus:ring-green-500"
                    autoFocus
                  />
                  <button
                    onClick={() => applyTrimStart(trimStartInstruction)}
                    className="px-4 py-2 rounded-lg text-sm font-medium bg-green-600 text-white hover:bg-green-700 transition-colors"
                  >
                    Set
                  </button>
                </div>
              </div>
            </div>

            {/* Dialog footer */}
            <div className="px-4 py-3 border-t bg-gray-50 rounded-b-xl text-xs text-gray-400">
              Press Escape to cancel, Enter to confirm
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default ReplayPage;
