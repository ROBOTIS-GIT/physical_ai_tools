#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim

"""
CLI script to convert ROSbag recordings with MP4 videos to LeRobot dataset format.

Supports both v2.1 and v3.0 formats.

Usage:
    # Convert a single rosbag (v2.1 format, default)
    python convert_rosbag_to_lerobot.py \\
        --input /path/to/rosbag \\
        --output /path/to/output_dataset \\
        --repo-id user/dataset_name

    # Convert to v3.0 format
    python convert_rosbag_to_lerobot.py \\
        --input /path/to/rosbag \\
        --output /path/to/output_dataset \\
        --repo-id user/dataset_name \\
        --version v3.0

    # Convert multiple rosbags
    python convert_rosbag_to_lerobot.py \\
        --input /path/to/rosbag1 /path/to/rosbag2 /path/to/rosbag3 \\
        --output /path/to/output_dataset \\
        --repo-id user/dataset_name

    # Convert all rosbags in a directory
    python convert_rosbag_to_lerobot.py \\
        --input-dir /path/to/rosbags_folder \\
        --output /path/to/output_dataset \\
        --repo-id user/dataset_name

    # With custom settings
    python convert_rosbag_to_lerobot.py \\
        --input /path/to/rosbag \\
        --output /path/to/output_dataset \\
        --repo-id user/dataset_name \\
        --fps 30 \\
        --robot-type ai_worker \\
        --no-trim \\
        --no-exclude

    # v3.0 with custom file sizes
    python convert_rosbag_to_lerobot.py \\
        --input /path/to/rosbag \\
        --output /path/to/output_dataset \\
        --repo-id user/dataset_name \\
        --version v3.0 \\
        --data-file-size 100 \\
        --video-file-size 200
"""

import argparse
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from physical_ai_server.data_processing import (
    RosbagToLerobotConverter,
    ConversionConfig,
)
from physical_ai_server.data_processing.rosbag_to_lerobot_v30_converter import (
    RosbagToLerobotV30Converter,
    V30ConversionConfig,
)


def setup_logging(verbose: bool = False) -> logging.Logger:
    """Setup logging configuration."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    return logging.getLogger(__name__)


def find_rosbags_in_directory(directory: Path) -> list[Path]:
    """Find all rosbag directories in a given directory."""
    rosbags = []

    for item in sorted(directory.iterdir()):
        if not item.is_dir():
            continue

        # Check if it's a rosbag directory (contains .mcap or .db3 files)
        mcap_files = list(item.glob("*.mcap"))
        db3_files = list(item.glob("*.db3"))

        if mcap_files or db3_files:
            rosbags.append(item)

    return rosbags


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROSbag recordings with MP4 videos to LeRobot dataset format (v2.1 or v3.0).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Single rosbag (v2.1 default)
    %(prog)s --input /data/rosbag_001 --output /datasets/my_dataset --repo-id user/my_dataset

    # Convert to v3.0 format
    %(prog)s --input /data/rosbag_001 --output /datasets/my_dataset --repo-id user/my_dataset --version v3.0

    # Multiple rosbags
    %(prog)s --input /data/rosbag_001 /data/rosbag_002 --output /datasets/my_dataset --repo-id user/my_dataset

    # All rosbags in a directory
    %(prog)s --input-dir /data/recordings --output /datasets/my_dataset --repo-id user/my_dataset
        """,
    )

    # Input options (mutually exclusive)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        "--input",
        "-i",
        nargs="+",
        type=Path,
        help="Path(s) to rosbag directories to convert",
    )
    input_group.add_argument(
        "--input-dir",
        "-d",
        type=Path,
        help="Directory containing multiple rosbag directories",
    )

    # Output options
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        required=True,
        help="Output directory for the LeRobot dataset",
    )
    parser.add_argument(
        "--repo-id",
        "-r",
        type=str,
        required=True,
        help="Repository ID for the dataset (e.g., 'user/dataset_name')",
    )

    # Version option
    parser.add_argument(
        "--version",
        type=str,
        choices=["v2.1", "v3.0"],
        default="v2.1",
        help="LeRobot dataset format version (default: v2.1)",
    )

    # Conversion options
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Target frames per second (default: 30)",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="unknown",
        help="Robot type identifier (default: 'unknown')",
    )
    parser.add_argument(
        "--chunks-size",
        type=int,
        default=1000,
        help="Maximum episodes per chunk (default: 1000)",
    )

    # v3.0 specific options
    parser.add_argument(
        "--data-file-size",
        type=int,
        default=100,
        help="[v3.0] Target data file size in MB (default: 100)",
    )
    parser.add_argument(
        "--video-file-size",
        type=int,
        default=200,
        help="[v3.0] Target video file size in MB (default: 200)",
    )

    # Trim and exclude options
    parser.add_argument(
        "--no-trim",
        action="store_true",
        help="Disable trim point application from robot_config.yaml",
    )
    parser.add_argument(
        "--no-exclude",
        action="store_true",
        help="Disable exclude region application from robot_config.yaml",
    )

    # Other options
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose logging",
    )

    args = parser.parse_args()

    # Setup logging
    logger = setup_logging(args.verbose)

    # Gather input paths
    if args.input:
        bag_paths = args.input
    else:
        if not args.input_dir.exists():
            logger.error(f"Input directory does not exist: {args.input_dir}")
            sys.exit(1)

        bag_paths = find_rosbags_in_directory(args.input_dir)
        if not bag_paths:
            logger.error(f"No rosbag directories found in: {args.input_dir}")
            sys.exit(1)

        logger.info(f"Found {len(bag_paths)} rosbag directories")

    # Validate input paths
    for path in bag_paths:
        if not path.exists():
            logger.error(f"Input path does not exist: {path}")
            sys.exit(1)

    logger.info("=" * 60)
    logger.info(f"ROSbag to LeRobot {args.version} Converter")
    logger.info("=" * 60)
    logger.info(f"Input rosbags: {len(bag_paths)}")
    for i, path in enumerate(bag_paths):
        logger.info(f"  [{i}] {path}")
    logger.info(f"Output directory: {args.output}")
    logger.info(f"Repository ID: {args.repo_id}")
    logger.info(f"Target FPS: {args.fps}")
    logger.info(f"Robot type: {args.robot_type}")
    logger.info(f"Apply trim: {not args.no_trim}")
    logger.info(f"Apply exclude regions: {not args.no_exclude}")
    if args.version == "v3.0":
        logger.info(f"Data file size: {args.data_file_size} MB")
        logger.info(f"Video file size: {args.video_file_size} MB")
    logger.info("=" * 60)

    if args.version == "v3.0":
        config = V30ConversionConfig(
            repo_id=args.repo_id,
            output_dir=args.output,
            fps=args.fps,
            robot_type=args.robot_type,
            chunks_size=args.chunks_size,
            apply_trim=not args.no_trim,
            apply_exclude_regions=not args.no_exclude,
            data_file_size_in_mb=args.data_file_size,
            video_file_size_in_mb=args.video_file_size,
        )
        converter = RosbagToLerobotV30Converter(config, logger)
    else:
        config = ConversionConfig(
            repo_id=args.repo_id,
            output_dir=args.output,
            fps=args.fps,
            robot_type=args.robot_type,
            chunks_size=args.chunks_size,
            apply_trim=not args.no_trim,
            apply_exclude_regions=not args.no_exclude,
        )
        converter = RosbagToLerobotConverter(config, logger)

    success = converter.convert_multiple_rosbags(bag_paths)

    if success:
        logger.info("=" * 60)
        logger.info("Conversion completed successfully!")
        logger.info(f"Dataset location: {args.output}")
        logger.info("=" * 60)
        sys.exit(0)
    else:
        logger.error("Conversion failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
