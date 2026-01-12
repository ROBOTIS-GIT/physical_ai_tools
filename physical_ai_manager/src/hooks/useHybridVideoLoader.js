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

import { useState, useEffect, useCallback, useRef } from 'react';

/**
 * Hybrid video loading hook that enables immediate playback via streaming
 * while downloading full video blobs in the background for smooth seeking.
 *
 * Loading flow:
 * 1. Start with direct streaming URLs (instant playback)
 * 2. Download blobs in background with progress tracking
 * 3. Switch to blob URLs when download completes
 *
 * @param {Object} params - Hook parameters
 * @param {string} params.bagPath - Current bag path
 * @param {Array<string>} params.videoFiles - List of video file names
 * @param {string} params.rosHost - ROS host address
 * @param {number} params.videoServerPort - Video server port
 * @param {boolean} params.isLoaded - Whether replay data is loaded
 */
export function useHybridVideoLoader({
    bagPath,
    videoFiles,
    rosHost,
    videoServerPort,
    isLoaded,
}) {
    // State
    const [streamingUrls, setStreamingUrls] = useState([]);
    const [blobUrls, setBlobUrls] = useState([]);
    const [isDownloading, setIsDownloading] = useState(false);
    const [downloadProgress, setDownloadProgress] = useState(0);
    const [isFullyLoaded, setIsFullyLoaded] = useState(false);
    const [loadingPhase, setLoadingPhase] = useState('idle'); // 'idle' | 'streaming' | 'downloading' | 'complete'

    // Cache for downloaded blobs (persists across bag switches within session)
    const blobCacheRef = useRef(new Map());

    // Abort controller for cancelling downloads
    const abortControllerRef = useRef(null);

    /**
     * Get the streaming URL for a video file (direct HTTP)
     */
    const getStreamingUrl = useCallback(
        (videoFile) => {
            if (!bagPath || !videoFile) return null;
            return `http://${rosHost}:${videoServerPort}/video/${bagPath}/${videoFile}`;
        },
        [bagPath, rosHost, videoServerPort]
    );

    /**
     * Download all videos as blobs in parallel
     */
    const downloadAllBlobs = useCallback(
        async () => {
            if (!videoFiles.length || !bagPath) return;

            // Check cache first
            const cachedUrls = blobCacheRef.current.get(bagPath);
            if (cachedUrls && cachedUrls.length === videoFiles.length) {
                setBlobUrls(cachedUrls);
                setIsFullyLoaded(true);
                setLoadingPhase('complete');
                setDownloadProgress(100);
                return;
            }

            // Cancel any existing download
            if (abortControllerRef.current) {
                abortControllerRef.current.abort();
            }
            abortControllerRef.current = new AbortController();

            setIsDownloading(true);
            setLoadingPhase('downloading');

            const totalVideos = videoFiles.length;
            const progressPerVideo = 100 / totalVideos;
            const newBlobUrls = [];
            const videoProgress = new Array(totalVideos).fill(0);

            try {
                // Download all videos in parallel
                const downloads = videoFiles.map(async (file, index) => {
                    const url = getStreamingUrl(file);
                    if (!url) return null;

                    // Use generator for progress updates
                    const response = await fetch(url, { signal: abortControllerRef.current.signal });
                    if (!response.ok) throw new Error(`Failed: ${response.status}`);

                    const contentLength = parseInt(response.headers.get('content-length') || '0', 10);
                    const reader = response.body.getReader();
                    const chunks = [];
                    let received = 0;

                    while (true) {
                        const { done, value } = await reader.read();
                        if (done) break;

                        chunks.push(value);
                        received += value.length;

                        // Update this video's progress
                        if (contentLength > 0) {
                            videoProgress[index] = (received / contentLength) * progressPerVideo;
                            const totalProgress = videoProgress.reduce((a, b) => a + b, 0);
                            setDownloadProgress(Math.round(totalProgress));
                        }
                    }

                    const blob = new Blob(chunks, { type: 'video/mp4' });
                    return URL.createObjectURL(blob);
                });

                const results = await Promise.all(downloads);
                newBlobUrls.push(...results.filter(Boolean));

                if (newBlobUrls.length === totalVideos) {
                    // Cache the blob URLs
                    blobCacheRef.current.set(bagPath, newBlobUrls);
                    setBlobUrls(newBlobUrls);
                    setIsFullyLoaded(true);
                    setLoadingPhase('complete');
                    setDownloadProgress(100);
                }
            } catch (error) {
                if (error.name !== 'AbortError') {
                    console.error('Video download failed:', error);
                }
            } finally {
                setIsDownloading(false);
            }
        },
        [videoFiles, bagPath, getStreamingUrl]
    );

    /**
     * Initialize streaming URLs immediately when replay data loads
     */
    useEffect(() => {
        if (!isLoaded || !videoFiles.length || !bagPath) {
            setStreamingUrls([]);
            setBlobUrls([]);
            setIsFullyLoaded(false);
            setLoadingPhase('idle');
            setDownloadProgress(0);
            return;
        }

        // Check if we have cached blobs
        const cachedUrls = blobCacheRef.current.get(bagPath);
        if (cachedUrls && cachedUrls.length === videoFiles.length) {
            setBlobUrls(cachedUrls);
            setStreamingUrls(cachedUrls); // Use blob URLs directly
            setIsFullyLoaded(true);
            setLoadingPhase('complete');
            setDownloadProgress(100);
            return;
        }

        // Set streaming URLs for immediate playback
        const urls = videoFiles.map(getStreamingUrl);
        setStreamingUrls(urls);
        setLoadingPhase('streaming');

        // Start background download
        const timeoutId = setTimeout(() => {
            downloadAllBlobs();
        }, 500); // Small delay to prioritize UI rendering

        return () => {
            clearTimeout(timeoutId);
            if (abortControllerRef.current) {
                abortControllerRef.current.abort();
            }
        };
    }, [isLoaded, videoFiles, bagPath, getStreamingUrl, downloadAllBlobs]);

    /**
     * Cleanup blob URLs on unmount
     */
    useEffect(() => {
        const cache = blobCacheRef.current;
        return () => {
            cache.forEach((urls) => {
                urls.forEach((url) => {
                    if (url && url.startsWith('blob:')) {
                        URL.revokeObjectURL(url);
                    }
                });
            });
            cache.clear();
        };
    }, []);

    /**
     * Get the best available URL for a video index.
     * Returns blob URL if available, otherwise streaming URL.
     */
    const getVideoUrl = useCallback(
        (index) => {
            if (isFullyLoaded && blobUrls[index]) {
                return blobUrls[index];
            }
            return streamingUrls[index] || null;
        },
        [isFullyLoaded, blobUrls, streamingUrls]
    );

    /**
     * Reset state when bag path changes
     */
    const reset = useCallback(() => {
        if (abortControllerRef.current) {
            abortControllerRef.current.abort();
        }
        setBlobUrls([]);
        setStreamingUrls([]);
        setIsFullyLoaded(false);
        setDownloadProgress(0);
        setLoadingPhase('idle');
        setIsDownloading(false);
    }, []);

    return {
        // URLs
        getVideoUrl,
        streamingUrls,
        blobUrls,

        // Status
        isDownloading,
        isFullyLoaded,
        downloadProgress,
        loadingPhase,

        // Actions
        reset,

        // For display
        hasVideos: videoFiles.length > 0,
        videoCount: videoFiles.length,
    };
}

export default useHybridVideoLoader;
