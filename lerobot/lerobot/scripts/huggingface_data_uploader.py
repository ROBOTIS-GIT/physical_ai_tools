import os
import subprocess
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

hf_token = "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"
subprocess.run(
    ["huggingface-cli", "login", "--token", hf_token, "--add-to-git-credential"],
    check=True,
)

hf_user = subprocess.run(
    ["huggingface-cli", "whoami"], capture_output=True, text=True
).stdout.split("\n")[0]

repo_id = f"{hf_user}/koch_test_500_episodes_2"
root = '/home/dc/.cache/huggingface/lerobot/RobotisSW/koch_test_500_episodes_2'
local_files_only=True
tags = '["tutorial"]'
private = False


dataset = LeRobotDataset(
            repo_id,
            root=root,
            local_files_only=local_files_only,
        )


dataset.push_to_hub(tags=tags, private=private)