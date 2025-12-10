^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package physical_ai_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.2 (2025-12-01)
------------------
* Fixed an issue where the task_index was being merged based on the first episode when merging episodes in the *.parquet data.
* Contributors: Dongyun Kim

0.7.1 (2025-11-28)
------------------
* Updated physical_ai_server launch file to use direct rosbridge websocket node instantiation
* Contributors: Kiwoong Park

0.7.0 (2025-11-21)
------------------
* Added rosbag_recorder package
* Added rosbag2 recording support when collecting LeRobot datasets
* Contributors: Woojin Wie, Kiwoong Park

0.6.13 (2025-10-27)
------------------
* Fixed physical_ai_server crash when querying user ID without locally registered HuggingFace token
* Changed to skip automatic HF user ID loading on Record page when Push to Hub is disabled
* Contributors: Kiwoong Park

0.6.12 (2025-10-21)
------------------
* Enhanced SendTrainingCommand.srv with resume functionality.
* Added GetTrainingInfo.srv for retrieving training configuration from saved model checkpoints.
* Added training resume functionality.
* Improved performance of dataset episode deletion by implementing batch deletion.
* Contributors: Seongwoo Kim, Kiwoong Park

0.6.11 (2025-09-30)
------------------
* Added Hugging Face upload & download functionality.
* Contributors: Dongyun Kim, Kiwoong Park

0.6.10 (2025-09-19)
------------------
* Prevent duplicate ROS2 services when changing robot type repeatedly.
* Added auto-reconnect subscriptions when setting robot type after physical_ai_server restart.
* Fixed a bug in the file browser component that caused multiple calls to the browseFile service.
* Contributors: Kiwoong Park

0.6.9 (2025-09-18)
------------------
* Changed omx_config.yaml file.
* Contributors: Junha Cha

0.6.8 (2025-08-21)
------------------
* Added DatasetInfo.msg, EditDataset.srv, and GetDatasetInfo.srv interfaces for Data Edit communication.
* Added UI features for editing datasets, including merge and delete functionality.
* Added functionality to edit data in the Physical AI Server, including merge and delete operations.
* Added ROS topics and services to receive parameters related to data editing.
* Contributors: Dongyun Kim, Kiwoong Park

0.6.7 (2025-08-18)
------------------
* Added a beep sound to signal the start of recording.
* Improved the convenience of data acquisition by using the AI Worker's buttons.
* The right button moves to the next episode, and the left button is for cancellation.
* Contributors: Dongyun Kim

0.6.6 (2025-08-13)
------------------
* Fixed an error in the data saving method based on Lerobot.
* Contributors: Dongyun Kim

0.6.5 (2025-08-11)
------------------
* Added file browse-related message type
* Added file browser component for policy selection in the inference page
* Added file browsing service with target file checking for policy path selection
* Contributors: Kiwoong Park

0.6.4 (2025-08-07)
------------------
* Added training loss display
* Added publishing of current loss during training
* Contributors: Kiwoong Park, Seongwoo Kim

0.6.3 (2025-07-25)
------------------
* Fixed a bug to allow setting the output folder path to a specified location.
* Fixed a bug that did not guarantee the order of messages.
* Contributors: Dongyun Kim, Seongwoo Kim, Woojin Wie

0.6.2 (2025-07-24)
------------------
* Updated Lerobot to the latest version and modified related functionalities.
* Contributors: Dongyun Kim, Seongwoo Kim, Woojin Wie

0.6.1 (2025-07-23)
------------------
* Implemented robust error handling during data collection to prevent server crashes due to incorrect robot type configuration
* Contributors: Seongwoo Kim

0.6.0 (2025-07-23)
------------------
* Added training-related service types and message types
* Added a new training page for training imitation learning models
* Implemented a Training Manager to support model training through the Web UI
* Contributors: Kiwoong Park, Seongwoo Kim

0.5.13 (2025-07-21)
------------------
* Updated Docker volume mount paths from `/root` to `${HOME}` for improved portability and compatibility
* Contributors: Seongwoo Kim

0.5.12 (2025-07-18)
------------------
* Enabled appending video encodings without overwriting existing data in multi-task mode
* Contributors: Seongwoo Kim

0.5.11 (2025-07-16)
------------------
* Added functionality for evaluating trained models
* Contributors: Dongyun Kim

0.5.10 (2025-07-15)
------------------
* Added multi-tasking data recording support to the Physical AI Server
* Contributors: Kiwoong Park, Seongwoo Kim

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
* Added heartbeat topic publishing to monitor alive status of Physical AI Server
* Contributors: Kiwoong Park, Dongyun Kim

0.5.7 (2025-06-26)
------------------
* Added Image Transport Plugin and fixed missing Gstreamer components
* Contributors: Dongyun Kim

0.5.6 (2025-06-26)
------------------
* Reordered pip install order in Dockerfile to fix the numpy version issue
* Contributors: Woojin Wie

0.5.5 (2025-06-26)
------------------
* Fixed control panel button states not reflecting correct taskType when switching between Record and Inference pages
* Contributors: Kiwoong Park

0.5.4 (2025-06-25)
------------------
* Added support for inference mode in the physical AI Server, including a new InferencePage and related UI components.
* Changed the robot naming format.
* Added Robot Config to support FFW-SG2 robot.
* Added Msg Topic and data acquisition functionality to support Mobile Robot.
* Fixed minor errors in the data acquisition process to improve software stability.
* Added a new inference page for running and monitoring inference tasks.
* Added inference-related msgs and srv types.
* Contributors: Dongyun Kim, Kiwoong Park

0.5.3 (2025-06-16)
------------------
* Refactored Physical AI Server for improved data collection capabilities
* Implemented data acquisition functionality using ROS2 topics
* Modified configuration system to allow flexible robot type selection
* Updated data collection method to utilize image buffers for efficiency
* Overall UI improvements for physical_ai_manager
* Added status information display from physical_ai_server
* Added functionality to receive task information from users and send commands to physical_ai_server
* Added bringup launch file that runs physical_ai_server with rosbridge_server and webvideo_server
* Contributors: Dongyun Kim, Kiwoong Park

0.5.2 (2025-05-29)
------------------
* Adjusted the waiting timeout for joint states.
* Contributors: Dongyun Kim

0.5.1 (2025-05-29)
------------------
* Added quality and transport parameters to image streaming URL
* Added a Docker setup for physical AI server
* Contributors: Kiwoong Park

0.5.0 (2025-05-20)
------------------
* Added a web UI for physical AI data collection
* Removed unnecessary dependencies and cleaned up the codebase
* Updated the LeRobot submodule to the latest version
* Refactored to a scalable structure that supports N cameras and various joint configurations
* Contributors: Dongyun Kim, Kiwoong Park, Woojin Wie, Seongwoo Kim

0.4.0 (2025-05-15)
------------------
* Added a pipeline for data collection and inference based on ROS2
* Refactored to a scalable structure that supports N cameras and various joint configurations
* Contributors: Dongyun Kim

0.3.1 (2025-05-08)
------------------
* Updated the LeRobot submodule to the latest version
* Contributors: Woojin Wie

0.3.0 (2025-04-25)
------------------
* Unified multiple launch files into a single configurable launch file for better usability
* Contributors: Seongwoo Kim

0.2.0 (2025-04-08)
------------------
* Added a time stamper node for data synchronization purposes
* Removed unused joints and motors bus config
* Contributors: Seongwoo Kim, Hyungyu Kim

0.1.0 (2025-04-07)
------------------
* Added a full workflow for recording and visualizing datasets using the LeRobot interface
* Added bringup scripts for system initialization
* Contributors: Seongwoo Kim, Pyo
