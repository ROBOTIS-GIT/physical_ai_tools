# Video Encoder Module - FEATURES

## Overview
Module for encoding image frames to MP4 video. Supports FFmpeg (CPU) and GStreamer (GPU) backends.

---

## Classes

### VideoEncoder (Abstract Base)
**File**: `encoder_base.py`

Abstract base class for all encoders.

#### Attributes
| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `fps` | int | 30 | Frame rate |
| `vcodec` | str | 'libx264' | Video codec |
| `pix_fmt` | str | 'yuv420p' | Pixel format |
| `g` | int | None | GOP size |
| `crf` | int | 23 | Quality (lower is better) |
| `qp` | int | None | QP value (for NVENC) |
| `fast_decode` | int | None | Fast decode option |
| `buffer` | list | [] | Frame buffer |

#### Abstract Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `encode_video` | video_path | - | Encode video |
| `is_encoding_completed` | - | bool | Check encoding complete |
| `get_encoding_status` | - | dict | Get encoding status |

#### Buffer Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `add_frame` | frame | - | Add frame |
| `clear_buffer` | - | - | Clear buffer |

---

### FFmpegEncoder
**File**: `ffmpeg_encoder.py`

CPU-based FFmpeg encoder. Supports chunk-based processing.

#### Additional Attributes
| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `chunk_size` | int | 100 | Frames per chunk |
| `preset` | str | 'medium' | Encoding preset |
| `clear_after_encode` | bool | True | Delete buffer after encoding |
| `is_encoding` | bool | False | Encoding in progress |
| `encoding_completed` | bool | False | Encoding complete |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `encode_video` | video_path | - | Encode video |
| `is_encoding_completed` | - | bool | Check encoding complete |
| `get_encoding_status` | - | dict | Get detailed encoding status |

#### Encoding Status
```python
{
    'is_encoding': bool,
    'encoding_completed': bool,
    'total_frames': int,
    'total_frames_encoded': int,
    'total_chunks': int,
    'chunks_encoded': int,
    'current_chunk': int,
    'progress_percentage': float,
    'output_path': str,
    'started_at': float,          # Optional
    'elapsed_time': float,        # Optional
    'finished_at': float,         # Optional
    'encoding_time': float,       # Optional
    'file_size': int,             # Optional (bytes)
    'file_size_kb': float,        # Optional
    'encoding_fps': float         # Optional
}
```

#### FFmpeg Presets
| Preset | Speed | Quality | Use Case |
|--------|-------|---------|----------|
| ultrafast | Very fast | Low | Quick preview |
| fast | Fast | Medium | General recording |
| medium | Medium | Good | Default |
| slow | Slow | High | Quality priority |

---

### GStreamerEncoder
**File**: `gstreamer_encoder.py`

NVIDIA GPU accelerated GStreamer encoder.

#### Additional Attributes
| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_nvenc` | bool | True | Use NVENC |
| `bitrate` | int | 4000000 | Bitrate (bps) |

---

## Supported Codecs

### CPU Codecs (FFmpeg)
| Codec | Description | Use Case |
|-------|-------------|----------|
| `libx264` | H.264 encoder | Good compatibility (default) |
| `libx265` | H.265 encoder | Good compression |
| `libsvtav1` | AV1 encoder | Latest, high efficiency |

### GPU Codecs (NVENC)
| Codec | Description | Use Case |
|-------|-------------|----------|
| `h264_nvenc` | NVIDIA H.264 | Fast encoding |
| `hevc_nvenc` | NVIDIA H.265 | Fast + high compression |

---

## Dependencies

### External
| Package | Components |
|---------|------------|
| `ffmpeg` | System FFmpeg binary |
| `subprocess` | Process execution |
| `gstreamer` (optional) | GPU encoding |

---

## Usage Examples

### FFmpeg Encoding
```python
from physical_ai_server.video_encoder.ffmpeg_encoder import FFmpegEncoder
import numpy as np

encoder = FFmpegEncoder(
    fps=30,
    vcodec='libx264',
    crf=23,
    preset='medium',
    chunk_size=100
)

# Add frames
for i in range(300):
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    encoder.add_frame(frame)

# Encode
encoder.encode_video('/path/to/output.mp4')

# Check status
status = encoder.get_encoding_status()
print(f"Progress: {status['progress_percentage']:.1f}%")
print(f"Encoding FPS: {status.get('encoding_fps', 0):.1f}")
```

### GStreamer Encoding (GPU)
```python
from physical_ai_server.video_encoder.gstreamer_encoder import GStreamerEncoder

encoder = GStreamerEncoder(
    fps=30,
    use_nvenc=True,
    bitrate=4000000
)

for frame in frames:
    encoder.add_frame(frame)

encoder.encode_video('/path/to/output.mp4')
```

---

## Notes
- FFmpegEncoder processes in chunks for memory efficiency
- Use `qp` parameter instead of `crf` for NVENC
- Buffer auto-deleted after encoding when `clear_after_encode=True`
- AV1 (`libsvtav1`) supports `fast-decode` option
