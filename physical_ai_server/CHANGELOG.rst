^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package physical_ai_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2025-07-23)
------------------
* Implemented a Training Manager to support model training through the Web UI
* Contributors: Seongwoo Kim

0.5.13 (2025-07-21)
------------------
* None

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
* Contributors: Seongwoo Kim

0.5.9 (2025-07-07)
------------------
* None

0.5.8 (2025-07-07)
------------------
* Added heartbeat topic publishing to monitor alive status of Physical AI Server
* Contributors: Dongyun Kim

0.5.7 (2025-06-26)
------------------
* Added Image Transport Plugin and fixed missing Gstreamer components
* Contributors: Dongyun Kim

0.5.6 (2025-06-26)
------------------
* None

0.5.5 (2025-06-26)
------------------
* None

0.5.4 (2025-06-25)
------------------
* Added support for inference mode in the physical AI Server, including a new InferencePage and related UI components.
* Changed the robot naming format.
* Added Robot Config to support FFW-SG2 robot.
* Added Msg Topic and data acquisition functionality to support Mobile Robot.
* Fixed minor errors in the data acquisition process to improve software stability.
* Contributors: Dongyun Kim

0.5.3 (2025-06-16)
------------------
* Refactored Physical AI Server for improved data collection capabilities
* Implemented data acquisition functionality using ROS2 topics
* Modified configuration system to allow flexible robot type selection
* Updated data collection method to utilize image buffers for efficiency
* Contributors: Dongyun Kim

0.5.2 (2025-05-29)
------------------
* None

0.5.1 (2025-05-29)
------------------
* None

0.5.0 (2025-05-20)
------------------
* Renamed physical_ai_manager to physical_ai_server.
* Contributors: Dongyun Kim

0.4.0 (2025-05-15)
------------------
* Added a pipeline for data collection and inference based on ROS2.
* Refactored to a scalable structure that supports N cameras and various joint configurations.
* Contributors: Dongyun Kim
