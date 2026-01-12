^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2026-01-08)
------------------
* Added FFmpeg-based image compression to MP4
* Image topics now store metadata in MCAP, frames in MP4 videos
* Support for sensor_msgs/Image and CompressedImage topics
* Added compress_images parameter for enabling/disabling compression
* Added ImageMetadata.msg for frame index and video path tracking
* Contributors: Dongyun Kim

0.7.2 (2025-12-01)
------------------
* None

0.7.1 (2025-11-28)
------------------
* None

0.7.0 (2025-11-21)
------------------
* Added rosbag2 recording preparation/start/stop/stop_and_delete/finish functionality
* Added ros2 service interface for rosbag2 recording
* Contributors: Woojin Wie, Kiwoong Park
