^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package physical_ai_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.2 (2025-12-01)
------------------
* None

0.7.1 (2025-11-28)
------------------
* None

0.7.0 (2025-11-21)
------------------
* Added rosbag2 record option to the Record page
* Contributors: Kiwoong Park

0.6.13 (2025-10-27)
------------------
* Changed to skip automatic HF user ID loading on Record page when Push to Hub is disabled
* Contributors: Kiwoong Park

0.6.12 (2025-10-21)
------------------
* Added UI features for training resume functionality.
* Contributors: Kiwoong Park

0.6.11 (2025-09-30)
------------------
* Added Hugging Face upload & download functionality to the Data Tools page.
* Added model download capability from Hugging Face in the inference page.
* Contributors: Kiwoong Park

0.6.10 (2025-09-19)
------------------
* Added auto-reconnect subscriptions when setting robot type after physical_ai_server restart.
* Fixed a bug in the file browser component that caused multiple calls to the browseFile service.
* Contributors: Kiwoong Park

0.6.9 (2025-09-18)
------------------
* None

0.6.8 (2025-08-21)
------------------
* Added UI features for editing datasets, including merge and delete functionality.
* Contributors: Kiwoong Park

0.6.7 (2025-08-18)
------------------
* Added a beep sound to signal the start of recording.
* Contributors: Dongyun Kim

0.6.6 (2025-08-13)
------------------
* None

0.6.5 (2025-08-11)
------------------
* Added file browser component for policy selection in the inference page
* Contributors: Kiwoong Park

0.6.4 (2025-08-07)
------------------
* Added training loss display
* Contributors: Kiwoong Park

0.6.3 (2025-07-25)
------------------
* None

0.6.2 (2025-07-24)
------------------
* None

0.6.1 (2025-07-23)
------------------
* None

0.6.0 (2025-07-23)
------------------
* Added a new training page for training imitation learning models
* Contributors: Kiwoong Park

0.5.13 (2025-07-21)
------------------
* None

0.5.12 (2025-07-18)
------------------
* None

0.5.11 (2025-07-16)
------------------
* None

0.5.10 (2025-07-15)
------------------
* Added multi-tasking data recording support in record page
* Contributors: Kiwoong Park

0.5.9 (2025-07-07)
------------------
* Use global ROS connection instead of multiple instances
* Add proper cleanup for image streams to prevent accumulation
* Remove unnecessary scrollbars in Chrome browser
* Contributors: Kiwoong Park

0.5.8 (2025-07-07)
------------------
* Applied Redux Toolkit for better state management
* Added heartbeat status to the UI
* Contributors: Kiwoong Park

0.5.7 (2025-06-26)
------------------
* None

0.5.6 (2025-06-26)
------------------
* None

0.5.5 (2025-06-26)
------------------
* Fixed control panel button states not reflecting correct taskType when switching between Record and Inference pages
* Contributors: Kiwoong Park

0.5.4 (2025-06-25)
------------------
* Added a new inference page for running and monitoring inference tasks
* Contributors: Kiwoong Park

0.5.3 (2025-06-16)
------------------
* Overall UI improvements for physical_ai_manager
* Added status information display from physical_ai_server
* Added functionality to receive task information from users and send commands to physical_ai_server
* Added bringup launch file that runs physical_ai_server with rosbridge_server and webvideo_server
* Contributors: Kiwoong Park

0.5.2 (2025-05-29)
------------------
* None

0.5.1 (2025-05-29)
------------------
* Added quality and transport parameters to image streaming URL
* Contributors: Kiwoong Park

0.5.0 (2025-05-20)
------------------
* Added a web UI for physical AI data collection
* Contributors: Kiwoong Park
