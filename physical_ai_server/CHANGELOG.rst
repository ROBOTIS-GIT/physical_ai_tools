^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package physical_ai_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2025-05-29)
------------------
* Added data saving functionality.
* Used an image buffer to reduce CPU overhead.
* Modified to dynamically load parameters based on robot name received from the service.
* Refactored the ROS Parameter structure to eliminate redundant data, improve variable naming, and create a more flexible structure.
* Added Save, Load, Upload, and Download functionality for data management.
* Contributors: Dongyun Kim

0.5.0 (2025-05-20)
------------------
* Renamed physical_ai_manager to physical_ai_server.
* Contributors: Dongyun Kim

0.4.0 (2025-05-15)
------------------
* Added a pipeline for data collection and inference based on ROS2.
* Refactored to a scalable structure that supports N cameras and various joint configurations.
* Contributors: Dongyun Kim
