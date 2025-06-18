^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package physical_ai_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
