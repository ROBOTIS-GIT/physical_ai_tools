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

import { useEffect, useRef } from 'react';

/**
 * Hook for handling keyboard shortcuts in Replay Viewer.
 * Uses refs to avoid stale closures in event handlers.
 *
 * @param {Object} params - Hook parameters
 * @param {boolean} params.isActive - Whether the page is active
 * @param {Object} params.handlers - Object containing handler functions
 */
export function useKeyboardShortcuts({
    isActive,
    handlers,
}) {
    // Refs to hold latest handler functions
    const handlersRef = useRef(handlers);

    // Update refs when handlers change
    useEffect(() => {
        handlersRef.current = handlers;
    }, [handlers]);

    useEffect(() => {
        if (!isActive) return;

        const handleKeyDown = (e) => {
            // Ignore when focused on input fields
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

            const h = handlersRef.current;

            // Using e.code for IME compatibility (works with Korean/Japanese input mode)
            switch (e.code) {
                // Navigation
                case 'ArrowUp':
                    e.preventDefault();
                    h.onNavigatePrev?.();
                    break;
                case 'ArrowDown':
                    e.preventDefault();
                    h.onNavigateNext?.();
                    break;

                // Seeking
                case 'ArrowLeft':
                    e.preventDefault();
                    if (e.shiftKey) {
                        h.onSeekRelative?.(-5);
                    } else {
                        h.onStepBackward?.();
                    }
                    break;
                case 'ArrowRight':
                    e.preventDefault();
                    if (e.shiftKey) {
                        h.onSeekRelative?.(5);
                    } else {
                        h.onStepForward?.();
                    }
                    break;

                // Playback
                case 'Space':
                    e.preventDefault();
                    h.onTogglePlayPause?.();
                    break;

                // A-B Loop
                case 'KeyA':
                    e.preventDefault();
                    h.onToggleLoopPoint?.();
                    break;
                case 'Backspace':
                    e.preventDefault();
                    h.onClearLoop?.();
                    break;

                // Task Markers
                case 'KeyM':
                    e.preventDefault();
                    h.onOpenMarkerDialog?.();
                    break;
                case 'KeyD':
                    e.preventDefault();
                    h.onDeleteNearestMarker?.();
                    break;

                // Jump to marker by number (Digit1 through Digit9)
                case 'Digit1':
                case 'Digit2':
                case 'Digit3':
                case 'Digit4':
                case 'Digit5':
                case 'Digit6':
                case 'Digit7':
                case 'Digit8':
                case 'Digit9':
                    e.preventDefault();
                    h.onJumpToMarker?.(parseInt(e.code.replace('Digit', '')) - 1);
                    break;

                // Trim points
                case 'KeyS':
                    e.preventDefault();
                    h.onSetTrimStart?.();
                    break;
                case 'KeyE':
                    e.preventDefault();
                    h.onSetTrimEnd?.();
                    break;

                // Exclude regions
                case 'KeyX':
                    e.preventDefault();
                    h.onToggleExcludeRegion?.();
                    break;

                // Help (Shift + /)
                case 'Slash':
                    if (e.shiftKey) {
                        e.preventDefault();
                        h.onToggleHelp?.();
                    }
                    break;

                // Escape for various actions
                case 'Escape':
                    e.preventDefault();
                    if (h.hasPendingExclude) {
                        h.onCancelExclude?.();
                    } else if (h.showHelpModal) {
                        h.onCloseHelp?.();
                    } else if (h.showTrimDialog) {
                        h.onCloseTrimDialog?.();
                    } else if (h.showMarkerDialog) {
                        h.onCloseMarkerDialog?.();
                    }
                    break;

                default:
                    break;
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [isActive]);

    return null;
}

/**
 * Default keyboard shortcuts help content
 */
export const KEYBOARD_SHORTCUTS = [
    { key: 'Space', description: 'Play/Pause' },
    { key: '←/→', description: 'Step frame backward/forward' },
    { key: 'Shift + ←/→', description: 'Seek ±5 seconds' },
    { key: '↑/↓', description: 'Previous/Next rosbag' },
    { key: 'A', description: 'Set A-B loop point' },
    { key: 'Backspace', description: 'Clear A-B loop' },
    { key: 'S', description: 'Set trim start point' },
    { key: 'E', description: 'Set trim end point' },
    { key: 'M', description: 'Add task marker' },
    { key: 'D', description: 'Delete nearest marker' },
    { key: '1-9', description: 'Jump to marker' },
    { key: 'X', description: 'Set exclude region (press twice)' },
    { key: 'Esc', description: 'Cancel/Close' },
    { key: '?', description: 'Toggle help' },
];

export default useKeyboardShortcuts;
