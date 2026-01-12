# Evaluation Module - FEATURES

## Overview
Module for evaluating trained policy performance and visualizing results. Supports MSE-based offline evaluation.

---

## Classes

### EvaluationManager
**File**: `evaluation_manager.py`

Policy evaluation pipeline manager.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `load_dataset` | repo_id, root, episodes | LeRobotDataset | Load dataset for evaluation |
| `get_episode_boundaries` | dataset, episode_idx | tuple[int, int, int] | Get episode start/end/length |
| `get_episode_data` | dataset, episode_idx | List[Dict] | Get all frames of episode |
| `extract_frame_components` | frame_data | tuple | Extract GT state, action, images, task |
| `calculate_mse` | gt_actions, pred_actions | float | Calculate MSE |
| `evaluate_policy_on_episode` | inference_manager, dataset, episode_idx, plot, save_plot_path | float | Evaluate single episode |
| `evaluate_policy_on_dataset` | inference_manager, dataset, sample_episodes, plot_episodes, plot_summary, save_plot_dir | dict | Evaluate entire dataset |

#### Evaluation Results
```python
{
    'mean_mse': float,          # Mean MSE
    'std_mse': float,           # MSE standard deviation
    'min_mse': float,           # Minimum MSE
    'max_mse': float,           # Maximum MSE
    'best_episode_idx': int,    # Best performing episode
    'worst_episode_idx': int,   # Worst performing episode
    'total_episodes': int,      # Number of evaluated episodes
    'episode_mses': List[float] # Per-episode MSE
}
```

---

### VisualizationManager
**File**: `visualization_manager.py`

Evaluation result visualization manager.

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `plot_action_comparison` | episode_idx, state, gt_action, pred_action, save_path, mse | - | GT vs Pred action comparison plot |
| `plot_episode_mse_comparison` | episode_mses, save_path, title | - | Per-episode MSE bar chart |
| `plot_episode_mse_distribution` | episode_mses, save_path | - | MSE distribution histogram |

---

## Dependencies

### Internal
| Module | Usage |
|--------|-------|
| `inference.inference_manager` | InferenceManager |

### External
| Package | Components |
|---------|------------|
| `lerobot` | LeRobotDataset, DatasetConfig |
| `matplotlib` | Plotting |
| `numpy` | Array operations |

---

## Usage Examples

### Single Episode Evaluation
```python
from physical_ai_server.evaluation.evaluation_manager import EvaluationManager
from physical_ai_server.inference.inference_manager import InferenceManager

eval_manager = EvaluationManager()
inference_manager = InferenceManager(device='cuda')

# Load dataset
dataset = eval_manager.load_dataset('user/dataset_name')

# Load policy
inference_manager.validate_policy('/path/to/policy')
inference_manager.load_policy()

# Evaluate episode
mse = eval_manager.evaluate_policy_on_episode(
    inference_manager=inference_manager,
    dataset=dataset,
    episode_idx=0,
    plot=True,
    save_plot_path='./episode_0_eval.png'
)
print(f'Episode 0 MSE: {mse}')
```

### Full Dataset Evaluation
```python
results = eval_manager.evaluate_policy_on_dataset(
    inference_manager=inference_manager,
    dataset=dataset,
    sample_episodes=[0, 1, 2, 3, 4],  # None = all
    plot_episodes=True,
    plot_summary=True,
    save_plot_dir='./evaluation_plots'
)

print(f"Mean MSE: {results['mean_mse']:.6f}")
print(f"Best Episode: {results['best_episode_idx']}")
```

### CLI Execution
```bash
python evaluation_manager.py \
    --repo_id user/dataset_name \
    --policy_path /path/to/policy \
    --sample_episodes 0 1 2 3 \
    --save_plot_dir ./plots
```

---

## Output Files

```
./plots/
├── episode_0_evaluation.png     # Per-episode GT vs Pred comparison
├── episode_1_evaluation.png
├── ...
├── overall_mse_comparison.png   # Per-episode MSE bar chart
└── overall_mse_distribution.png # MSE distribution histogram
```

---

## Notes
- Only offline evaluation supported (no environment simulation)
- MSE calculated across all action dimensions
- Images converted from [C, H, W] tensor to [H, W, C] numpy for inference
