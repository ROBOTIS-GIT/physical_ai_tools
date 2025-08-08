#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Dongyun Kim, Seongwoo Kim

from typing import List, Optional, Any
import time


class ActionChunkProcessor:
    """
    Handles action chunk processing including offset application, 
    trajectory interpolation, and robot state analysis.
    """
    
    def __init__(self, logger=None):
        self.logger = logger
        self.gap_threshold = 0.3  # About 17 degrees in radians
        self.position_threshold = 0.02  # Threshold for static detection
        
    def apply_offset_and_smoothing(
        self, 
        action_chunk: List[List[float]], 
        actions_executed_during_inference: int,
        action_history: Any,
        last_executed_action: Optional[List[float]]
    ) -> List[List[float]]:
        """
        Apply smart offset based on robot state to prevent discontinuous motion.
        If robot was static during inference, use all actions (offset=0).
        If robot was moving, use time-based offset.
        """
        
        # Analyze robot state during inference time
        is_robot_static_during_inference = self.was_robot_static_during_inference(
            actions_executed_during_inference, action_history, last_executed_action)
        
        # Calculate smart offset - simplified logic
        if is_robot_static_during_inference:
            smart_offset = 0
            if self.logger:
                self.logger.info('Static robot during inference: Using all actions (offset=0)')
        else:
            smart_offset = actions_executed_during_inference
            if self.logger:
                self.logger.info(f'Moving robot during inference: Using time-based offset ({smart_offset})')
        
        # Apply smart offset
        if smart_offset > 0:
            if smart_offset < len(action_chunk):
                offset_action_chunk = action_chunk[smart_offset:]
                if self.logger:
                    self.logger.info(f'Applied offset: skipped first {smart_offset} actions')
            else:
                if self.logger:
                    self.logger.warning(
                        f'Offset ({smart_offset}) exceeds chunk length ({len(action_chunk)})!')
                offset_action_chunk = []
        else:
            offset_action_chunk = action_chunk.copy()
            if self.logger:
                self.logger.info(f'No offset applied - using all {len(action_chunk)} actions')

        # Apply trajectory interpolation if needed to smooth transitions
        if (
                last_executed_action is not None and
                offset_action_chunk and
                len(offset_action_chunk) > 0
            ):
            # Check if trajectory interpolation is needed
            first_new_action = offset_action_chunk[0]
            max_position_diff = max(
                abs(first_new_action[i] - last_executed_action[i]) 
                for i in range(len(last_executed_action))
            )
            
            if max_position_diff > self.gap_threshold:
                # Generate interpolated actions to bridge the gap
                interpolated_actions = self.generate_interpolated_actions(
                    last_executed_action, first_new_action, max_position_diff)
                
                if interpolated_actions:
                    # Insert interpolated actions at the beginning
                    offset_action_chunk = interpolated_actions + offset_action_chunk
                    if self.logger:
                        self.logger.info(
                            f'Applied trajectory interpolation: added {len(interpolated_actions)} bridge actions '
                            f'for gap of {max_position_diff:.4f}')
                else:
                    if self.logger:
                        self.logger.info(
                            f'Gap detected ({max_position_diff:.4f}) but no interpolation generated')
            else:
                if self.logger:
                    self.logger.debug(f'No interpolation needed - gap is small ({max_position_diff:.4f})')
        
        return offset_action_chunk

    def generate_interpolated_actions(
        self, 
        start_action: List[float], 
        end_action: List[float], 
        gap_size: float
    ) -> List[List[float]]:
        """
        Generate interpolated actions to create smooth trajectory between two actions.
        """
        try:
            # Determine number of interpolation steps based on gap size
            if gap_size > 0.8:  # Very large gap (>45 degrees)
                num_steps = 9  # Large gap - more steps
            elif gap_size > 0.5:  # Large gap (>30 degrees)
                num_steps = 6  # Medium gap
            elif gap_size > 0.3:  # Medium gap (>17 degrees)
                num_steps = 3  # Small gap
            else:
                return []  # No interpolation needed

            interpolated_actions = []

            for step in range(1, num_steps + 1):
                # Linear interpolation factor (0 to 1)
                t = step / (num_steps + 1)

                # Create interpolated action
                interpolated_action = []
                for i in range(len(start_action)):
                    # Linear interpolation between start and end
                    interpolated_value = start_action[i] + t * (end_action[i] - start_action[i])
                    interpolated_action.append(interpolated_value)

                interpolated_actions.append(interpolated_action)
            
            return interpolated_actions

        except Exception as e:
            if self.logger:
                self.logger.error(f'Error generating interpolated actions: {str(e)}')
            return []

    def was_robot_static_during_inference(
        self,
        actions_executed_during_inference: int,
        action_history: Any,
        last_executed_action: Optional[List[float]]
    ) -> bool:
        """
        Check if robot was static during the inference period by analyzing 
        the actions executed during that time.
        """
        if actions_executed_during_inference <= 0:
            if self.logger:
                self.logger.info('Robot was static during inference: No actions executed')
            return True  # No actions executed, so robot was static

        if len(action_history) < actions_executed_during_inference:
            if self.logger:
                self.logger.info(
                    f'Robot static analysis: Not enough history ({len(action_history)} < {actions_executed_during_inference})')
            return False  # Not enough history to analyze

        # Get actions executed during inference time
        inference_period_actions = list(action_history)[-actions_executed_during_inference:]

        if len(inference_period_actions) < 2:
            if self.logger:
                self.logger.info('Robot was static during inference: Too few actions to analyze')
            return True  # Too few actions to determine motion

        # Calculate position variations during inference period
        max_variation = 0.0
        joint_variations = []
        if last_executed_action is not None:
            for joint_idx in range(len(last_executed_action)):
                joint_positions = [action['action_values'][joint_idx] for action in inference_period_actions]
                joint_max = max(joint_positions)
                joint_min = min(joint_positions)
                variation = joint_max - joint_min
                joint_variations.append(variation)
                max_variation = max(max_variation, variation)

        is_static = max_variation < self.position_threshold

        # Detailed logging for debugging
        if self.logger:
            self.logger.info(
                f'Static analysis during inference ({actions_executed_during_inference} actions): '
                f'is_static={is_static}, max_variation={max_variation:.6f}, threshold={self.position_threshold:.6f}')

            if len(joint_variations) > 0:
                self.logger.info(
                    f'Joint variations: {[f"{v:.6f}" for v in joint_variations[:3]]}...')  # Show first 3 joints

            # Show actual position values for debugging
            if len(inference_period_actions) > 0 and last_executed_action is not None:
                first_action_pos = inference_period_actions[0]['action_values'][:3]  # First 3 joints
                last_action_pos = inference_period_actions[-1]['action_values'][:3]   # First 3 joints
                self.logger.info(
                    f'Position range - First: {[f"{p:.6f}" for p in first_action_pos]}, '
                    f'Last: {[f"{p:.6f}" for p in last_action_pos]}')

        return is_static

    def set_gap_threshold(self, threshold: float):
        """Set the threshold for trajectory interpolation gap detection."""
        self.gap_threshold = threshold
        
    def set_position_threshold(self, threshold: float):
        """Set the threshold for static robot detection."""
        self.position_threshold = threshold
