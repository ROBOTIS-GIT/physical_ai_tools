^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package physical_ai_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
