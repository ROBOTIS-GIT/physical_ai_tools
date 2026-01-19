# Rosbag Timing Visualization Tool

Comprehensive timing analysis and visualization tool for ROS2 rosbag recordings. Generates detailed timeline plots showing message arrival patterns, gaps, and anomalies.

## Overview

The timing visualization tool helps developers verify and analyze the timing characteristics of rosbag recordings, particularly useful for:
- Verifying zero-delay recording implementations
- Detecting dropped or delayed messages
- Analyzing multi-topic synchronization
- Validating frequency consistency
- Identifying timing anomalies

## Features

### Visualization
- **Timeline Plots**: X-axis timestamp, Y-axis topics
- **Message Markers**: Color-coded by topic type
- **Gap Detection**: Red annotations for gaps exceeding threshold
- **Automatic Segmentation**: Configurable time segments (default 5s)
- **10ms Grid Lines**: Fine-grained timing verification
- **Legend & Statistics**: Embedded analysis panel

### Analysis
- **Frequency Calculation**: Actual publishing frequency per topic
- **Gap Statistics**: Min/max/average/std dev of message intervals
- **Anomaly Detection**: Identify delays, gaps, and bursts
- **Multi-topic Synchronization**: Verify temporal alignment
- **JSON Reports**: Machine-readable analysis results

### Output
- **PNG Visualization**: High-quality timeline image
- **JSON Report**: Detailed statistics for programmatic analysis
- **Text Summary**: Console output with key findings

## Installation

### Requirements
```bash
python3 >= 3.8
matplotlib >= 3.5
numpy >= 1.20
rosbags >= 0.9.0
```

### Setup
```bash
pip install matplotlib numpy rosbags
```

### Docker
Already installed in `physical_ai_server` container:
```bash
docker exec physical_ai_server python3 /root/ros2_ws/src/physical_ai_tools/tools/visualize_timing.py
```

## Usage

### Basic Usage
```bash
python3 visualize_timing.py /path/to/rosbag
```

Generates:
- `rosbag_timing.png` (visualization)
- `rosbag_timing_report.json` (statistics)

### With Custom Output Path
```bash
python3 visualize_timing.py /path/to/rosbag \
  -o /tmp/my_timeline.png \
  -r /tmp/my_report.json
```

### Custom Segment Duration (2 seconds)
```bash
python3 visualize_timing.py /path/to/rosbag -s 2.0
```

Finer-grained timeline with more segments.

### Custom Gap Threshold (50ms)
```bash
python3 visualize_timing.py /path/to/rosbag -g 50.0
```

Stricter anomaly detection - flags gaps > 50ms.

### Combined Options
```bash
python3 visualize_timing.py /workspace/test_bag \
  -o /workspace/timing.png \
  -r /workspace/timing_report.json \
  -s 3.0 \
  -g 75.0 \
  -d /workspace/output
```

## Command-Line Options

```
positional arguments:
  bag_path              Path to rosbag directory (required)

optional arguments:
  -o, --output PATH     Output PNG file path
  -r, --report PATH     Output JSON report file path
  -d, --output-dir PATH Output directory for all files
  -s, --segment-duration SEC
                        Timeline segment duration (default: 5.0)
  -g, --gap-threshold MS
                        Gap detection threshold in milliseconds (default: 100.0)
```

## Output Files

### PNG Visualization
**File**: `{bagname}_timing.png`
**Content**:
- Timeline segments stacked vertically
- Message dots at exact timestamps
- Red dashed lines for gap anomalies
- Gap duration labels
- Statistics panel with key metrics

**Example dimensions**: 3419 x 3470 pixels for 36.8s recording
**File size**: ~1.2 MB

### JSON Report
**File**: `{bagname}_timing_report.json`
**Structure**:
```json
{
  "bag_path": "...",
  "timestamp_ms": 36847,
  "total_topics": 11,
  "segment_duration_s": 5.0,
  "gap_threshold_ms": 100.0,
  "topics": {
    "TOPIC_NAME": {
      "message_type": "...",
      "message_count": 1234,
      "duration_s": 36.8,
      "frequency_hz": 33.5,
      "first_timestamp_s": 0.021,
      "last_timestamp_s": 36.847,
      "gap_statistics": {
        "min_ms": 7.76,
        "max_ms": 78.30,
        "avg_ms": 33.30,
        "std_dev_ms": 5.92
      },
      "anomalies": {
        "count": 0,
        "threshold_ms": 100.0,
        "gaps": []
      }
    }
  }
}
```

### Console Output
Text summary with:
- Topics found and message counts
- Frequency analysis
- Gap statistics
- Anomalies detected

## Interpreting Results

### Visualization
1. **Message Spacing**: Consistent dot spacing indicates uniform frequency
2. **Gaps**: Red lines indicate missed data or delays
3. **Multiple Topics**: Aligned dots show good synchronization
4. **Frequency Verification**: Compare visual spacing against expected frequency

### Statistics
1. **Frequency**: Should match expected topic publishing rate
2. **Gap Min/Max**: Expected range for uniform periodic publishing
   - 100Hz topics: ~10ms average, 0-20ms range
   - 30Hz topics: ~33ms average, 20-50ms range
3. **Std Dev**: Lower values indicate more consistent timing
4. **Anomalies**: Count of gaps exceeding threshold (should be 0)

## Examples

### Verify Zero-Delay Recording
```bash
# Create visualization with standard 100ms gap threshold
python3 visualize_timing.py /workspace/test_bag/episode_0 \
  -o /workspace/zero_delay_verification.png

# Expected results:
# - First message at < 10ms
# - No red gap lines
# - Uniform message spacing
```

### High-Frequency Analysis
```bash
# Use smaller segment duration for detailed view
python3 visualize_timing.py /workspace/high_freq_bag \
  -s 1.0 \
  -o /workspace/detailed_timeline.png

# Smaller 1-second segments reveal fine timing variations
```

### Strict Anomaly Detection
```bash
# Lower gap threshold for stricter analysis
python3 visualize_timing.py /workspace/bag \
  -g 50.0 \
  -o /workspace/strict_analysis.png

# Marks any gap > 50ms as anomaly (red line)
```

### Automated Testing Integration
```bash
# Generate both image and JSON for programmatic analysis
python3 visualize_timing.py /workspace/test_output/test_bag \
  -o /workspace/test_output/timing.png \
  -r /workspace/test_output/timing_stats.json

# Parse JSON to verify test assertions
python3 << 'EOF'
import json
with open('/workspace/test_output/timing_stats.json') as f:
    stats = json.load(f)

for topic, data in stats['topics'].items():
    assert data['anomalies']['count'] == 0, f"{topic} has gaps"
    assert data['frequency_hz'] > 29.0, f"{topic} frequency too low"
    print(f"✓ {topic} passed verification")
EOF
```

## Configuration

Edit `timing_config.yaml` to customize:
- Default segment duration
- Gap threshold
- Color scheme
- Figure dimensions
- Display options
- Message type mapping

## Performance

### Tested Performance
- Input: 36.8s recording, 11 topics, 24,129 messages
- Processing time: < 5 seconds
- Memory usage: Minimal (all data in memory)
- Output size: ~1.2 MB PNG + 6.7 KB JSON

### Scalability
- Supports 1-100+ topics
- Handles bags of any duration (auto-segments)
- Efficient timestamp extraction and sorting

## Troubleshooting

### ImportError: rosbags not found
```bash
pip install rosbags
```

### File not found error
Ensure bag_path is absolute path to rosbag directory:
```bash
# Correct
python3 visualize_timing.py /workspace/test_bag/0

# Incorrect
python3 visualize_timing.py ./test_bag/0  # relative path
```

### Visualization appears crowded
Use smaller segment duration:
```bash
python3 visualize_timing.py bag -s 2.0  # 2-second segments
```

### Too many anomalies detected
Increase gap threshold:
```bash
python3 visualize_timing.py bag -g 200.0  # 200ms threshold
```

### No anomalies but gaps visible
May be within threshold. Check JSON report for actual gap values:
```bash
# Check 'gap_statistics' in JSON for min/max/avg
grep -A 5 "gap_statistics" timing_report.json
```

## Integration with Test Suite

### In Test Scripts
```bash
#!/bin/bash

# Run test
python3 run_test.py

# Visualize results
python3 /path/to/visualize_timing.py \
  /workspace/test_output/recorded_bag \
  -o /workspace/test_output/timing_viz.png

# Check results
if grep '"count": 0' /workspace/test_output/timing_report.json; then
    echo "✓ No timing anomalies detected"
else
    echo "✗ Timing anomalies found"
    exit 1
fi
```

### Automated Verification
```python
import json
import subprocess

# Generate visualization
subprocess.run([
    'python3', 'visualize_timing.py',
    '/path/to/bag',
    '-r', 'report.json'
])

# Verify results
with open('report.json') as f:
    report = json.load(f)

success = all(
    t['anomalies']['count'] == 0
    for t in report['topics'].values()
)

if success:
    print("✓ All timing checks passed")
else:
    print("✗ Timing anomalies detected")
```

## Limitations

1. **Large Bags**: Bags > 1GB may require significant memory
2. **Many Topics**: 50+ topics may cause crowded visualization
3. **Text Overlap**: Gap labels may overlap in dense recordings
4. **Color Palette**: Limited to 10 unique colors (repeats after)

**Workarounds**:
- Use segment duration flag for coarser view
- Filter topics of interest
- Adjust figure size in config
- Increase margin in code if needed

## Future Enhancements

- Interactive Plotly visualization
- Topic filtering by regex
- Frequency domain analysis (FFT)
- Histogram of gap distributions
- Real-time streaming analysis
- Video export capability
- Multi-bag comparison

## API Reference

### TimingVisualizer Class

```python
from visualize_timing import TimingVisualizer

# Create visualizer
viz = TimingVisualizer(
    bag_path='/path/to/bag',
    segment_duration=5.0,      # seconds
    gap_threshold_ms=100.0,    # milliseconds
    output_dir='/tmp/output'
)

# Analyze bag
viz.analyze_bag()
viz.compute_statistics()

# Generate outputs
viz.visualize('/tmp/timeline.png')
viz.generate_report('/tmp/report.json')
viz.print_summary()
```

### Key Methods

- `analyze_bag()`: Extract timestamps from rosbag
- `compute_statistics()`: Calculate topic statistics
- `detect_gaps(topic)`: Find gaps > threshold
- `visualize(path)`: Generate PNG visualization
- `generate_report(path)`: Create JSON report
- `print_summary()`: Display text summary

## License

Part of Physical AI Tools project. See main LICENSE file.

## Support

For issues or feature requests, visit:
https://github.com/RobotisSW/physical_ai_tools

## Changelog

### Version 1.0.0 (2026-01-18)
- Initial release
- PNG timeline visualization
- JSON statistical report
- Configurable gap detection
- Multi-topic analysis
- Console summary output
