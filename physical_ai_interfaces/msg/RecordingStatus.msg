########################################
# Constants
########################################

string task_name
uint8 phase      # (0: NONE, 1: ON_WARM_UP, 2: ON_RESET 3: ON_EPISODE)
uint8 progress   # (0 ~ 100)
uint16 num_episodes
uint16 current_episode_number 

# phase
uint8 NONE = 0
uint8 ON_WARM_UP = 1
uint8 ON_RESET = 2
uint8 ON_EPISODE = 3