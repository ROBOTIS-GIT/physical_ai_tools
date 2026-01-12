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

/**
 * LTTB (Largest Triangle Three Buckets) downsampling algorithm.
 * Preserves visual shape while reducing data points for efficient rendering.
 *
 * @param {Array} data - Array of data points
 * @param {number} threshold - Target number of points after downsampling
 * @param {string} xKey - Key for x-axis value in data objects
 * @param {string} yKey - Key for y-axis value in data objects
 * @returns {Array} Downsampled data array
 */
export function lttbDownsample(data, threshold, xKey, yKey) {
    if (threshold >= data.length || threshold === 0) {
        return data;
    }

    const sampled = [];
    const dataLength = data.length;

    // Always include first point
    sampled.push(data[0]);

    // Bucket size
    const bucketSize = (dataLength - 2) / (threshold - 2);

    let a = 0; // Previously selected point index

    for (let i = 0; i < threshold - 2; i++) {
        // Calculate bucket boundaries
        const bucketStart = Math.floor((i + 1) * bucketSize) + 1;
        const bucketEnd = Math.min(Math.floor((i + 2) * bucketSize) + 1, dataLength);

        // Calculate average point in next bucket
        const avgRangeStart = Math.floor((i + 2) * bucketSize) + 1;
        const avgRangeEnd = Math.min(Math.floor((i + 3) * bucketSize) + 1, dataLength);

        let avgX = 0;
        let avgY = 0;
        let avgCount = 0;

        for (let j = avgRangeStart; j < avgRangeEnd; j++) {
            if (j < dataLength) {
                avgX += data[j][xKey];
                avgY += data[j][yKey] || 0;
                avgCount++;
            }
        }

        if (avgCount > 0) {
            avgX /= avgCount;
            avgY /= avgCount;
        }

        // Find point in current bucket that creates largest triangle
        let maxArea = -1;
        let maxAreaIndex = bucketStart;

        const pointA = data[a];

        for (let j = bucketStart; j < bucketEnd && j < dataLength; j++) {
            // Calculate triangle area using cross product
            const area = Math.abs(
                (pointA[xKey] - avgX) * ((data[j][yKey] || 0) - pointA[yKey]) -
                (pointA[xKey] - data[j][xKey]) * (avgY - pointA[yKey])
            );

            if (area > maxArea) {
                maxArea = area;
                maxAreaIndex = j;
            }
        }

        sampled.push(data[maxAreaIndex]);
        a = maxAreaIndex;
    }

    // Always include last point
    sampled.push(data[dataLength - 1]);

    return sampled;
}

/**
 * Prepare chart data from timestamps, names, and values arrays.
 * Applies LTTB downsampling for large datasets.
 *
 * @param {Array<number>} timestamps - Array of timestamp values
 * @param {Array<string>} names - Array of data field names (e.g., joint names)
 * @param {Array<number>} values - Flattened array of values (length = timestamps.length * names.length)
 * @param {string} prefix - Prefix for data keys (e.g., 'state_' or 'action_')
 * @param {number} targetPoints - Target number of points after downsampling (default: 1000)
 * @returns {Array} Chart-ready data array with time and prefixed value keys
 */
export function prepareChartData(timestamps, names, values, prefix, targetPoints = 1000) {
    if (!timestamps.length || !names.length || !values.length) {
        return [];
    }

    const numFields = names.length;

    // Create full data array
    const fullData = timestamps.map((time, i) => {
        const point = { time };
        const startIdx = i * numFields;
        names.forEach((name, j) => {
            point[`${prefix}${name}`] = values[startIdx + j] || 0;
        });
        return point;
    });

    // If data is small enough, return as is
    if (fullData.length <= targetPoints) {
        return fullData;
    }

    // Apply LTTB using the first field as reference
    const firstKey = `${prefix}${names[0]}`;
    return lttbDownsample(fullData, targetPoints, 'time', firstKey);
}

/**
 * Format file size to human readable format.
 *
 * @param {number} bytes - File size in bytes
 * @returns {string} Formatted file size string (e.g., "1.5 MB")
 */
export function formatFileSize(bytes) {
    if (bytes === 0 || !bytes) return '0 B';
    const units = ['B', 'KB', 'MB', 'GB', 'TB'];
    const i = Math.floor(Math.log(bytes) / Math.log(1024));
    return `${(bytes / Math.pow(1024, i)).toFixed(1)} ${units[i]}`;
}

/**
 * Format datetime to locale string (Korean format).
 *
 * @param {string} isoString - ISO format datetime string
 * @returns {string} Formatted datetime string
 */
export function formatDateTime(isoString) {
    if (!isoString) return '-';
    try {
        const date = new Date(isoString);
        return date.toLocaleString('ko-KR', {
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
        });
    } catch {
        return isoString;
    }
}

/**
 * Format time in seconds to MM:SS format.
 *
 * @param {number} seconds - Time in seconds
 * @returns {string} Formatted time string (e.g., "01:30")
 */
export function formatTime(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}
