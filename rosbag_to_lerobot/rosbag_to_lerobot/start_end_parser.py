"""Parser for start_end_points.jsonl files.

Each episode directory may contain a start_end_points.jsonl file with entries like:
    {"timestamp":"2026-03-03T01:45:30.334880616Z","label":"start"}
    {"timestamp":"2026-03-03T01:46:17.760216224Z","label":"end"}

Multiple start-end pairs produce multiple sub-episodes.
"""

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Tuple


def iso8601_to_epoch_seconds(iso_str: str) -> float:
    """Convert ISO 8601 timestamp string to epoch seconds.

    Handles nanosecond precision by truncating to microseconds
    (Python datetime limitation, negligible at 10-30 Hz).
    """
    # Remove trailing 'Z' and handle nanosecond precision
    s = iso_str.rstrip('Z')
    # Split into datetime part and fractional seconds
    if '.' in s:
        dt_part, frac = s.split('.', 1)
        # Truncate to 6 digits (microseconds)
        frac = frac[:6].ljust(6, '0')
        s = f"{dt_part}.{frac}"
    dt = datetime.strptime(s, "%Y-%m-%dT%H:%M:%S.%f")
    dt = dt.replace(tzinfo=timezone.utc)
    return dt.timestamp()


def parse_start_end_points(episode_dir: Path, logger=None) -> Optional[List[Tuple[float, float]]]:
    """Parse start_end_points.jsonl and return list of (start, end) time ranges.

    Returns:
        None: File does not exist (use full episode, backwards compatible)
        []: File exists but is invalid or empty (skip episode)
        [(s1, e1), ...]: Valid time ranges in epoch seconds
    """
    jsonl_path = episode_dir / "start_end_points.jsonl"
    if not jsonl_path.exists():
        return None

    def warn(msg):
        if logger:
            logger.warning(msg)

    # Read and parse all entries
    entries = []
    try:
        with open(jsonl_path, 'r') as f:
            lines = f.readlines()
    except IOError as e:
        warn(f"Failed to read {jsonl_path}: {e}")
        return []

    if not lines:
        warn(f"Empty start_end_points.jsonl in {episode_dir}")
        return []

    for i, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue
        try:
            entry = json.loads(line)
        except json.JSONDecodeError as e:
            warn(f"Invalid JSON at line {i + 1} in {jsonl_path}: {e}")
            return []

        if 'timestamp' not in entry or 'label' not in entry:
            warn(f"Missing 'timestamp' or 'label' at line {i + 1} in {jsonl_path}")
            return []

        label = entry['label']
        if label not in ('start', 'end'):
            warn(f"Unknown label '{label}' at line {i + 1} in {jsonl_path}")
            return []

        try:
            epoch_sec = iso8601_to_epoch_seconds(entry['timestamp'])
        except (ValueError, KeyError) as e:
            warn(f"Invalid timestamp at line {i + 1} in {jsonl_path}: {e}")
            return []

        entries.append((epoch_sec, label))

    if not entries:
        warn(f"No valid entries in {jsonl_path}")
        return []

    # Check even count
    if len(entries) % 2 != 0:
        warn(f"Odd number of entries ({len(entries)}) in {jsonl_path}. "
             f"Expected matching start-end pairs.")
        return []

    # Validate alternating start-end pattern in file order
    for i, (ts, label) in enumerate(entries):
        expected = 'start' if i % 2 == 0 else 'end'
        if label != expected:
            warn(f"Expected '{expected}' but got '{label}' at line {i + 1} "
                 f"in {jsonl_path}. "
                 f"Entries must alternate start-end in file order.")
            return []

    # Build pairs
    pairs = []
    for i in range(0, len(entries), 2):
        start_ts = entries[i][0]
        end_ts = entries[i + 1][0]
        if start_ts >= end_ts:
            warn(f"start ({start_ts}) >= end ({end_ts}) in pair {i // 2} "
                 f"of {jsonl_path}. Skipping this pair.")
            continue
        pairs.append((start_ts, end_ts))

    if not pairs:
        warn(f"No valid start-end pairs in {jsonl_path}")
        return []

    return pairs
