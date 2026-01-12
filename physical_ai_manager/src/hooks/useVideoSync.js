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

import { useRef, useCallback, useEffect } from 'react';
import { useDispatch } from 'react-redux';
import { setCurrentTime, setIsPlaying } from '../features/replay/replaySlice';

/**
 * Hook for synchronizing multiple video elements.
 * Handles play/pause, seek, A-B loop, and time updates.
 *
 * @param {Object} params - Hook parameters
 * @param {number} params.videoCount - Number of videos to sync
 * @param {number} params.duration - Total video duration
 * @param {number} params.currentTime - Current playback time (from Redux)
 * @param {boolean} params.isPlaying - Playing state (from Redux)
 * @param {number} params.playbackSpeed - Current playback speed
 * @param {number|null} params.loopStart - A-B loop start point
 * @param {number|null} params.loopEnd - A-B loop end point
 * @param {number|null} params.expandedVideoIndex - Currently expanded video index
 * @param {Array<number>} params.fps - FPS array for each video
 */
export function useVideoSync({
    videoCount,
    duration,
    currentTime,
    isPlaying,
    playbackSpeed = 1,
    loopStart = null,
    loopEnd = null,
    expandedVideoIndex = null,
    fps = [30],
}) {
    const dispatch = useDispatch();
    const videoRefs = useRef([]);

    /**
     * Get video element at index
     */
    const getVideo = useCallback((index) => {
        return videoRefs.current[index];
    }, []);

    /**
     * Set video ref at index
     */
    const setVideoRef = useCallback((index, element) => {
        videoRefs.current[index] = element;
        // Apply playback speed when ref is set
        if (element) {
            element.playbackRate = playbackSpeed;
        }
    }, [playbackSpeed]);

    /**
     * Get all valid video elements
     */
    const getAllVideos = useCallback(() => {
        return videoRefs.current.filter(Boolean);
    }, []);

    /**
     * Sync all videos to a specific time
     */
    const syncToTime = useCallback((time) => {
        const clampedTime = Math.max(0, Math.min(duration, time));
        getAllVideos().forEach((video) => {
            video.currentTime = clampedTime;
        });
        dispatch(setCurrentTime(clampedTime));
    }, [duration, getAllVideos, dispatch]);

    /**
     * Play all videos
     */
    const playAll = useCallback(() => {
        const videos = getAllVideos();
        const firstVideo = videos[0];

        // If video ended, restart from beginning
        if (firstVideo?.ended) {
            videos.forEach((v) => { v.currentTime = 0; });
            dispatch(setCurrentTime(0));
        } else {
            // Sync all videos to first video's time
            const targetTime = firstVideo?.currentTime || 0;
            videos.forEach((v) => { v.currentTime = targetTime; });
        }

        // Apply playback speed and play
        videos.forEach((v) => {
            v.playbackRate = playbackSpeed;
            v.play().catch(() => { });
        });
        dispatch(setIsPlaying(true));
    }, [getAllVideos, playbackSpeed, dispatch]);

    /**
     * Pause all videos
     */
    const pauseAll = useCallback(() => {
        getAllVideos().forEach((v) => v.pause());
        dispatch(setIsPlaying(false));
    }, [getAllVideos, dispatch]);

    /**
     * Toggle play/pause
     */
    const togglePlayPause = useCallback(() => {
        if (isPlaying) {
            pauseAll();
        } else {
            playAll();
        }
    }, [isPlaying, pauseAll, playAll]);

    /**
     * Restart playback from beginning
     */
    const restart = useCallback(() => {
        const videos = getAllVideos();
        videos.forEach((v) => {
            v.currentTime = 0;
            v.playbackRate = playbackSpeed;
            v.play().catch(() => { });
        });
        dispatch(setCurrentTime(0));
        dispatch(setIsPlaying(true));
    }, [getAllVideos, playbackSpeed, dispatch]);

    /**
     * Step one frame forward or backward
     */
    const stepFrame = useCallback((direction) => {
        // Pause if playing
        if (isPlaying) {
            pauseAll();
        }

        const currentFps = fps[0] || 30;
        const frameTime = 1 / currentFps;
        const delta = direction === 'forward' ? frameTime : -frameTime;
        const newTime = Math.max(0, Math.min(duration, currentTime + delta));

        getAllVideos().forEach((v) => {
            v.currentTime = newTime;
        });
        dispatch(setCurrentTime(newTime));
    }, [isPlaying, fps, duration, currentTime, getAllVideos, pauseAll, dispatch]);

    /**
     * Seek relative time (e.g., ±5 seconds)
     */
    const seekRelative = useCallback((seconds) => {
        const newTime = Math.max(0, Math.min(duration, currentTime + seconds));
        syncToTime(newTime);
    }, [duration, currentTime, syncToTime]);

    /**
     * Handle click on progress bar to seek
     */
    const handleProgressClick = useCallback((e, containerElement) => {
        if (!containerElement) return;

        const rect = containerElement.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const percentage = x / rect.width;
        const newTime = percentage * duration;
        syncToTime(newTime);
    }, [duration, syncToTime]);

    /**
     * Set playback speed for all videos
     */
    const setPlaybackSpeed = useCallback((speed) => {
        getAllVideos().forEach((v) => {
            v.playbackRate = speed;
        });
    }, [getAllVideos]);

    /**
     * Handle time update event from video
     */
    const handleTimeUpdate = useCallback(() => {
        const videos = getAllVideos();
        // Get time from first video or expanded video
        const sourceVideo = expandedVideoIndex !== null
            ? videos[expandedVideoIndex]
            : videos.find(v => v);

        if (sourceVideo) {
            dispatch(setCurrentTime(sourceVideo.currentTime));
        }
    }, [getAllVideos, expandedVideoIndex, dispatch]);

    /**
     * Handle video ended event
     */
    const handleEnded = useCallback(() => {
        dispatch(setIsPlaying(false));
        dispatch(setCurrentTime(duration));
    }, [duration, dispatch]);

    /**
     * A-B Loop: Jump to A when reaching B
     */
    useEffect(() => {
        if (!isPlaying || loopStart === null || loopEnd === null) return;

        if (currentTime >= loopEnd) {
            getAllVideos().forEach((v) => {
                v.currentTime = loopStart;
            });
            dispatch(setCurrentTime(loopStart));
        }
    }, [currentTime, isPlaying, loopStart, loopEnd, getAllVideos, dispatch]);

    /**
     * Apply playback speed when it changes
     */
    useEffect(() => {
        setPlaybackSpeed(playbackSpeed);
    }, [playbackSpeed, setPlaybackSpeed]);

    return {
        // Refs
        videoRefs,
        setVideoRef,
        getVideo,
        getAllVideos,

        // Actions
        togglePlayPause,
        playAll,
        pauseAll,
        restart,
        stepFrame,
        seekRelative,
        syncToTime,
        handleProgressClick,
        setPlaybackSpeed,

        // Event handlers (to be attached to videos)
        handleTimeUpdate,
        handleEnded,
    };
}

export default useVideoSync;
