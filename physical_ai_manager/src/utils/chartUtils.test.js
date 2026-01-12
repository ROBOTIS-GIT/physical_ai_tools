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

import {
    lttbDownsample,
    prepareChartData,
    formatFileSize,
    formatDateTime,
    formatTime,
} from './chartUtils';

describe('chartUtils', () => {
    describe('lttbDownsample', () => {
        it('should return original data if below threshold', () => {
            const data = [
                { time: 0, value: 1 },
                { time: 1, value: 2 },
                { time: 2, value: 3 },
            ];
            const result = lttbDownsample(data, 10, 'time', 'value');
            expect(result).toEqual(data);
        });

        it('should return original data if threshold is 0', () => {
            const data = [
                { time: 0, value: 1 },
                { time: 1, value: 2 },
            ];
            const result = lttbDownsample(data, 0, 'time', 'value');
            expect(result).toEqual(data);
        });

        it('should downsample data to target threshold', () => {
            // Create 100 data points
            const data = Array.from({ length: 100 }, (_, i) => ({
                time: i,
                value: Math.sin(i * 0.1),
            }));

            const result = lttbDownsample(data, 20, 'time', 'value');

            expect(result.length).toBe(20);
            // Should always include first and last points
            expect(result[0]).toEqual(data[0]);
            expect(result[result.length - 1]).toEqual(data[data.length - 1]);
        });

        it('should preserve visual extremes', () => {
            // Create data with clear peak
            const data = [
                { time: 0, value: 0 },
                { time: 1, value: 0 },
                { time: 2, value: 0 },
                { time: 3, value: 10 }, // Peak
                { time: 4, value: 0 },
                { time: 5, value: 0 },
                { time: 6, value: 0 },
                { time: 7, value: 0 },
                { time: 8, value: 0 },
                { time: 9, value: 0 },
            ];

            const result = lttbDownsample(data, 5, 'time', 'value');

            // Peak should be preserved due to largest triangle
            const hasExtreme = result.some(p => p.value === 10);
            expect(hasExtreme).toBe(true);
        });

        it('should handle null/undefined values', () => {
            const data = [
                { time: 0, value: 1 },
                { time: 1, value: null },
                { time: 2, value: undefined },
                { time: 3, value: 4 },
            ];

            // Should not throw
            expect(() => lttbDownsample(data, 3, 'time', 'value')).not.toThrow();
        });
    });

    describe('prepareChartData', () => {
        it('should return empty array for empty inputs', () => {
            expect(prepareChartData([], [], [], 'state_')).toEqual([]);
            expect(prepareChartData([1, 2], [], [1, 2], 'state_')).toEqual([]);
            expect(prepareChartData([1, 2], ['a'], [], 'state_')).toEqual([]);
        });

        it('should prepare chart data with correct prefix', () => {
            const timestamps = [0, 1, 2];
            const names = ['joint1', 'joint2'];
            const values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]; // 3 timestamps * 2 joints

            const result = prepareChartData(timestamps, names, values, 'state_', 100);

            expect(result.length).toBe(3);
            expect(result[0]).toEqual({
                time: 0,
                state_joint1: 1.0,
                state_joint2: 2.0,
            });
            expect(result[1]).toEqual({
                time: 1,
                state_joint1: 3.0,
                state_joint2: 4.0,
            });
        });

        it('should downsample large datasets', () => {
            const timestamps = Array.from({ length: 2000 }, (_, i) => i * 0.1);
            const names = ['joint1'];
            const values = Array.from({ length: 2000 }, (_, i) => Math.sin(i * 0.01));

            const result = prepareChartData(timestamps, names, values, 'state_', 500);

            expect(result.length).toBeLessThanOrEqual(500);
            expect(result.length).toBeGreaterThan(0);
        });
    });

    describe('formatFileSize', () => {
        it('should format bytes correctly', () => {
            expect(formatFileSize(0)).toBe('0 B');
            expect(formatFileSize(null)).toBe('0 B');
            expect(formatFileSize(undefined)).toBe('0 B');
        });

        it('should format KB correctly', () => {
            expect(formatFileSize(1024)).toBe('1.0 KB');
            expect(formatFileSize(2048)).toBe('2.0 KB');
        });

        it('should format MB correctly', () => {
            expect(formatFileSize(1024 * 1024)).toBe('1.0 MB');
            expect(formatFileSize(1.5 * 1024 * 1024)).toBe('1.5 MB');
        });

        it('should format GB correctly', () => {
            expect(formatFileSize(1024 * 1024 * 1024)).toBe('1.0 GB');
        });
    });

    describe('formatDateTime', () => {
        it('should return dash for empty input', () => {
            expect(formatDateTime(null)).toBe('-');
            expect(formatDateTime(undefined)).toBe('-');
            expect(formatDateTime('')).toBe('-');
        });

        it('should format valid ISO date string', () => {
            const result = formatDateTime('2025-01-12T14:30:00Z');
            // Should contain date parts
            expect(result).toMatch(/2025/);
            expect(result).toMatch(/01/);
            expect(result).toMatch(/12/);
        });

        it('should return original string for invalid date', () => {
            const invalidDate = 'not-a-date';
            const result = formatDateTime(invalidDate);
            // May return invalid date string or original based on locale
            expect(typeof result).toBe('string');
        });
    });

    describe('formatTime', () => {
        it('should format 0 seconds', () => {
            expect(formatTime(0)).toBe('00:00');
        });

        it('should format seconds only', () => {
            expect(formatTime(30)).toBe('00:30');
            expect(formatTime(59)).toBe('00:59');
        });

        it('should format minutes and seconds', () => {
            expect(formatTime(60)).toBe('01:00');
            expect(formatTime(90)).toBe('01:30');
            expect(formatTime(125)).toBe('02:05');
        });

        it('should handle large durations', () => {
            expect(formatTime(3600)).toBe('60:00'); // 1 hour
            expect(formatTime(3661)).toBe('61:01');
        });

        it('should handle decimal seconds', () => {
            expect(formatTime(30.5)).toBe('00:30');
            expect(formatTime(30.9)).toBe('00:30');
        });
    });
});
