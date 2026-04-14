########################################
# Constants
########################################
# phase
uint8 READY = 0
uint8 WARMING_UP = 1
uint8 RESETTING = 2
uint8 RECORDING = 3
uint8 SAVING = 4
uint8 STOPPED = 5
uint8 INFERENCING = 6
uint8 CONVERTING = 7
uint8 LOADING = 8
uint8 PAUSED = 9

TaskInfo task_info
string robot_type
uint8 phase                     # (0: READY, 1: WARMING_UP, 2: RESETTING, 3: RECORDING, 4: SAVING, 5: STOPPED, 6: INFERENCING, 7: CONVERTING, 8: LOADING, 9: PAUSED)
uint16 total_time               # [s]
uint16 proceed_time             # [s]
uint16 current_episode_number
uint16 current_scenario_number
string current_task_instruction
uint16 current_segment_index
uint16 segment_count
string[] segment_primitives
string merge_status
float32 encoding_progress       # [%]
float32 used_storage_size       # [GB]
float32 total_storage_size      # [GB]
float32 used_cpu                # [%]
float32 used_ram_size           # [GB]
float32 total_ram_size          # [GB]
string error
