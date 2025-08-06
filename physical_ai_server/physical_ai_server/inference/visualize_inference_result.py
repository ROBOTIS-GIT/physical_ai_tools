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
# Author: Dongyun Kim

import matplotlib.pyplot as plt
import numpy as np
import traceback


class InferenceResultVisualizer:
    """
    Handles visualization and analysis of inference results for action chunks.
    Provides tools to plot and analyze the relationship between inference outputs
    and actual robot execution.
    """
    
    def __init__(self, logger=None):
        """
        Initialize the visualizer.
        
        Args:
            logger: Logger instance for output messages
        """
        self.logger = logger

    def plot_action_chunk_curves(self, raw_action_chunks, action_history, inference_history):
        """
        Plot comprehensive action chunk curves showing inference outputs vs actual execution.
        
        Args:
            raw_action_chunks: List of raw action chunk data with metadata
            action_history: History of executed actions
            inference_history: History of inference timing and offset data
            
        Returns:
            str: Path to saved plot file, or None if failed
        """
        try:
            if len(raw_action_chunks) < 2:
                if self.logger:
                    self.logger.info("Not enough chunks to plot (need at least 2)")
                return None
                
            # Determine how many joints we have
            joint_count = raw_action_chunks[0]['joint_count'] if raw_action_chunks else 7
            joint_count = min(joint_count, 3)  # Limit to 3 joints to avoid layout issues

            # Calculate figure size to accommodate all subplots properly
            subplot_height = 3.5  # Height per subplot
            total_height = max(subplot_height * joint_count + 2, 10)  # Minimum 10 inches

            fig, axes = plt.subplots(joint_count, 1, figsize=(16, total_height), sharex=True)
            if joint_count == 1:
                axes = [axes]

            fig.suptitle(f'Action Chunk Analysis: Comprehensive View ({len(raw_action_chunks)} chunks)\n' +
                        'Inference Outputs vs Actual Execution (○: Inference Request, ×: Inference Complete)', 
                        fontsize=14, y=0.98)
            
            # Plot actual executed actions as continuous line
            if action_history:
                action_numbers = [a['action_number'] for a in action_history]
                
                for joint_idx in range(joint_count):
                    ax = axes[joint_idx]
                    
                    # Plot actual executed actions - ensure data consistency
                    if len(action_history) > 0 and len(action_history[0]['action_values']) > joint_idx:
                        actual_values = []
                        valid_action_numbers = []
                        
                        for action_data in action_history:
                            if len(action_data['action_values']) > joint_idx:
                                actual_values.append(action_data['action_values'][joint_idx])
                                valid_action_numbers.append(action_data['action_number'])
                        
                        if len(valid_action_numbers) == len(actual_values) and len(actual_values) > 0:
                            ax.plot(valid_action_numbers, actual_values, 'k-', linewidth=2, 
                                   label='Actual Executed Actions', alpha=0.8)
                    
                    # Plot each inference chunk as separate colored curves with extended color palette
                    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 
                             'olive', 'cyan', 'magenta', 'yellow', 'navy', 'lime', 'teal', 'maroon']
                    
                    for chunk_idx, chunk_data in enumerate(raw_action_chunks):
                        inference_start_step = chunk_data['inference_start_step']
                        raw_actions = chunk_data['raw_actions']
                        
                        if not raw_actions or len(raw_actions[0]) <= joint_idx:
                            continue
                            
                        # Get offset information from inference history
                        applied_offset = 0
                        used_chunk_size = len(raw_actions)
                        if chunk_idx < len(inference_history):
                            applied_offset = inference_history[chunk_idx]['calculated_offset']
                            used_chunk_size = inference_history[chunk_idx].get('used_chunk_size', len(raw_actions))
                        
                        # Only plot the ACTUALLY USED portion of the chunk (after offset)
                        if applied_offset < len(raw_actions):
                            used_actions = raw_actions[applied_offset:applied_offset + used_chunk_size]
                            
                            if not used_actions:
                                continue
                                
                            # Extract joint values for the used portion only
                            joint_values = [action[joint_idx] for action in used_actions]
                            
                            # Create step numbers for the actually used actions
                            actual_start_step = inference_start_step + applied_offset
                            chunk_steps = list(range(actual_start_step, 
                                                   actual_start_step + len(joint_values)))
                            
                            color = colors[chunk_idx % len(colors)]
                            ax.plot(chunk_steps, joint_values, color=color, linestyle='--', 
                                   linewidth=1.5, alpha=0.7, 
                                   label=f'Inference {chunk_idx+1} (used: {len(joint_values)}/{len(raw_actions)} actions)')
                            
                            # Mark the start point (when inference was requested)
                            ax.scatter([inference_start_step], [raw_actions[0][joint_idx]], 
                                     color=color, s=50, marker='o', alpha=0.8,
                                     label=f'Inf{chunk_idx+1} Request' if chunk_idx < 3 else None)
                            
                            # Mark the actual usage start point (after offset)
                            if applied_offset > 0:
                                ax.scatter([actual_start_step], [joint_values[0]], 
                                         color=color, s=50, marker='>', alpha=0.8,
                                         label=f'Inf{chunk_idx+1} Used Start' if chunk_idx < 3 else None)
                            
                            # Mark completion point (when this chunk was processed)
                            if chunk_idx < len(inference_history):
                                completion_action = inference_history[chunk_idx]['action_count_when_completed']
                                ax.scatter([completion_action], [raw_actions[min(applied_offset, len(raw_actions)-1)][joint_idx]], 
                                         color=color, s=50, marker='x', alpha=0.8,
                                         label=f'Inf{chunk_idx+1} Complete' if chunk_idx < 3 else None)
                    
                    ax.set_ylabel(f'Joint {joint_idx} Value')
                    ax.grid(True, alpha=0.3)
                    
                    # Limit legend entries to avoid overcrowding
                    handles, labels = ax.get_legend_handles_labels()
                    if len(handles) > 12:  # Limit to 12 entries for 10 chunks
                        handles = handles[:12]
                        labels = labels[:12]
                        labels[-1] = f"... and {len(handles) - 11} more"
                    
                    ax.legend(handles, labels, bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
                    
                    # Add vertical lines for chunk boundaries
                    if valid_action_numbers:  # Only if we have valid data
                        for chunk_data in raw_action_chunks:
                            start_step = chunk_data['inference_start_step']
                            if start_step <= max(valid_action_numbers):
                                ax.axvline(x=start_step, color='gray', linestyle=':', alpha=0.5)
            
            if joint_count > 0:
                axes[-1].set_xlabel('Action Step Number')
            
            # Use subplots_adjust for better control
            plt.subplots_adjust(left=0.08, right=0.75, top=0.95, bottom=0.08, hspace=0.3)
            
            # Save plot
            plot_path = f'/tmp/action_chunk_curves_{len(raw_action_chunks)}.png'
            plt.savefig(plot_path, dpi=120, bbox_inches='tight', pad_inches=0.2)
            plt.close()
            
            if self.logger:
                self.logger.info(f'Action chunk curves plot saved to: {plot_path}')
            
            return plot_path
            
        except Exception as e:
            if self.logger:
                self.logger.error(f'Error creating action chunk curves plot: {str(e)}')
                self.logger.error(f'Traceback: {traceback.format_exc()}')
            return None

    def plot_action_chunk_analysis(self, inference_history, action_history, chunk_visualization_data):
        """
        Plot detailed action chunk analysis for offset calculation debugging.
        
        Args:
            inference_history: History of inference timing and offset data
            action_history: History of executed actions
            chunk_visualization_data: Additional visualization data
            
        Returns:
            str: Path to saved plot file, or None if failed
        """
        try:
            if len(inference_history) < 2:
                if self.logger:
                    self.logger.info("Not enough inference history to plot analysis (need at least 2)")
                return None
                
            # Use larger figure size for better layout
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
            fig.suptitle('Action Chunk Analysis - Offset Calculation Debug', fontsize=14)
            
            # Extract data for plotting
            chunk_ids = list(range(len(inference_history)))
            inference_starts = [h['inference_start_action'] for h in inference_history]
            completion_actions = [h['action_count_when_completed'] for h in inference_history]
            calculated_offsets = [h['calculated_offset'] for h in inference_history]
            original_sizes = [h['original_chunk_size'] for h in inference_history]
            used_sizes = [h['used_chunk_size'] for h in inference_history]
            
            # Plot 1: Inference timing vs action execution
            ax1.plot(chunk_ids, inference_starts, 'b-o', label='Inference Start Action', markersize=4)
            ax1.plot(chunk_ids, completion_actions, 'r-s', label='Action Count at Completion', markersize=4)
            ax1.set_xlabel('Chunk ID')
            ax1.set_ylabel('Action Number')
            ax1.set_title('Inference Timing vs Action Execution')
            ax1.legend(fontsize=8)
            ax1.grid(True, alpha=0.3)
            
            # Plot 2: Offset calculation
            ax2.bar(chunk_ids, calculated_offsets, alpha=0.7, color='orange')
            ax2.set_xlabel('Chunk ID')
            ax2.set_ylabel('Calculated Offset')
            ax2.set_title('Applied Offsets for Each Chunk')
            ax2.grid(True, alpha=0.3)
            
            # Add text annotations for offsets
            for i, offset in enumerate(calculated_offsets):
                if offset > 0:
                    ax2.text(i, offset + 0.5, str(offset), ha='center', va='bottom', fontsize=8)
            
            # Plot 3: Chunk sizes (original vs used)
            x_pos = np.arange(len(chunk_ids))
            width = 0.35
            ax3.bar(x_pos - width/2, original_sizes, width, label='Original Size', alpha=0.7, color='lightblue')
            ax3.bar(x_pos + width/2, used_sizes, width, label='Used Size (after offset)', alpha=0.7, color='darkblue')
            ax3.set_xlabel('Chunk ID')
            ax3.set_ylabel('Chunk Size')
            ax3.set_title('Chunk Sizes: Original vs Used')
            ax3.set_xticks(x_pos)
            ax3.set_xticklabels(chunk_ids)
            ax3.legend(fontsize=8)
            ax3.grid(True, alpha=0.3)
            
            # Plot 4: Action values over time (first 3 dimensions)
            if action_history:
                action_numbers = [a['action_number'] for a in action_history]
                action_vals_0 = [a['action_values'][0] if len(a['action_values']) > 0 else 0 for a in action_history]
                action_vals_1 = [a['action_values'][1] if len(a['action_values']) > 1 else 0 for a in action_history]
                action_vals_2 = [a['action_values'][2] if len(a['action_values']) > 2 else 0 for a in action_history]
                
                ax4.plot(action_numbers, action_vals_0, 'r-', alpha=0.7, label='Joint 0', linewidth=1)
                ax4.plot(action_numbers, action_vals_1, 'g-', alpha=0.7, label='Joint 1', linewidth=1)
                ax4.plot(action_numbers, action_vals_2, 'b-', alpha=0.7, label='Joint 2', linewidth=1)
                
                # Mark chunk boundaries
                chunk_starts = chunk_visualization_data.get('chunk_start_actions', [])
                for start in chunk_starts:
                    if start <= max(action_numbers):
                        ax4.axvline(x=start, color='black', linestyle='--', alpha=0.5)
                
                ax4.set_xlabel('Action Number')
                ax4.set_ylabel('Joint Values')
                ax4.set_title('Action Values Over Time (with chunk boundaries)')
                ax4.legend(fontsize=8)
                ax4.grid(True, alpha=0.3)
            
            # Use subplots_adjust for better control
            plt.subplots_adjust(left=0.08, right=0.95, top=0.92, bottom=0.08, wspace=0.3, hspace=0.3)
            
            # Save plot
            plot_path = f'/tmp/action_chunk_analysis_{len(inference_history)}.png'
            plt.savefig(plot_path, dpi=120, bbox_inches='tight', pad_inches=0.2)
            plt.close()
            
            if self.logger:
                self.logger.info(f'Action chunk analysis plot saved to: {plot_path}')
            
            return plot_path
            
        except Exception as e:
            if self.logger:
                self.logger.error(f'Error creating action chunk analysis plot: {str(e)}')
                self.logger.error(f'Traceback: {traceback.format_exc()}')
            return None

    def print_chunk_curve_analysis(self, raw_action_chunks):
        """
        Print analysis of action chunk curves.
        
        Args:
            raw_action_chunks: List of raw action chunk data with metadata
        """
        if not self.logger:
            return
            
        self.logger.info("=== ACTION CHUNK CURVE ANALYSIS ===")
        
        for chunk_idx, chunk_data in enumerate(raw_action_chunks[-3:], 
                                             start=max(0, len(raw_action_chunks)-3)):
            inference_start = chunk_data['inference_start_step']
            raw_actions = chunk_data['raw_actions']
            joint_count = chunk_data['joint_count']
            
            self.logger.info(
                f"Chunk {chunk_idx}: Started from step {inference_start}, "
                f"Generated {len(raw_actions)} actions for {joint_count} joints"
            )
            
            # Show first and last action values
            if raw_actions:
                first_action = raw_actions[0][:3]  # First 3 joints
                last_action = raw_actions[-1][:3]  # First 3 joints
                self.logger.info(f"  First action: {first_action}")
                self.logger.info(f"  Last action:  {last_action}")
        
        self.logger.info("=== END CHUNK CURVE ANALYSIS ===")

    def print_offset_analysis(self, inference_history):
        """
        Print detailed offset analysis to help debug issues.
        
        Args:
            inference_history: History of inference timing and offset data
        """
        if not self.logger:
            return
            
        self.logger.info("=== DETAILED OFFSET ANALYSIS ===")
        
        # Print recent history (last 3 chunks)
        recent_chunks = inference_history[-3:] if len(inference_history) >= 3 else inference_history
        
        for i, history in enumerate(recent_chunks, start=max(0, len(inference_history)-len(recent_chunks))):
            inference_start = history['inference_start_action']
            completion_at = history['action_count_when_completed']
            calc_offset = history['calculated_offset']
            orig_size = history['original_chunk_size']
            used_size = history['used_chunk_size']
            actual_start = history.get('action_start_from', 'Unknown')
            
            # Calculate expected values
            expected_next_action = inference_start + calc_offset + 1
            actions_during_inference = completion_at - inference_start
            
            self.logger.info(
                f"Chunk {i}: "
                f"InfStart={inference_start}, "
                f"CompletionAt={completion_at}, "
                f"ActionsDuring={actions_during_inference}, "
                f"CalcOffset={calc_offset}, "
                f"OrigSize={orig_size}, "
                f"UsedSize={used_size}, "
                f"ExpectedNext={expected_next_action}, "
                f"ActualStart={actual_start}"
            )
            
            # Detailed analysis
            if calc_offset != actions_during_inference:
                self.logger.warning(
                    f"CALCULATION MISMATCH in Chunk {i}: "
                    f"Calculated offset ({calc_offset}) != Actions during inference ({actions_during_inference})"
                )
            
            # Check if offset makes sense
            if calc_offset >= orig_size:
                self.logger.warning(
                    f"OFFSET TOO LARGE in Chunk {i}: "
                    f"Offset ({calc_offset}) >= Original chunk size ({orig_size}), "
                    f"all actions would be skipped!"
                )
            
            # Check for discrepancy in expected vs actual
            if actual_start != 'Unknown' and expected_next_action != actual_start:
                difference = actual_start - expected_next_action
                self.logger.warning(
                    f"OFFSET MISMATCH in Chunk {i}: "
                    f"Expected next action {expected_next_action}, "
                    f"but actually started from {actual_start}. "
                    f"Difference: {difference} actions"
                )
                
                if difference < 0:
                    self.logger.error(f"CRITICAL: Actions being REPEATED! Going backwards by {abs(difference)} actions")
                elif difference > 0:
                    self.logger.warning(f"Actions being SKIPPED! Missing {difference} actions")
        
        # Summary statistics
        if len(inference_history) > 1:
            avg_offset = sum(h['calculated_offset'] for h in inference_history) / len(inference_history)
            avg_completion_time = sum(h['action_count_when_completed'] - h['inference_start_action'] 
                                    for h in inference_history) / len(inference_history)
            
            self.logger.info(
                f"SUMMARY: Total chunks: {len(inference_history)}, "
                f"Avg offset: {avg_offset:.1f}, "
                f"Avg actions during inference: {avg_completion_time:.1f}"
            )
        
        self.logger.info("=== END OFFSET ANALYSIS ===")

    def detect_offset_patterns(self, inference_history):
        """
        Detect patterns in offset calculation that might explain issues.
        
        Args:
            inference_history: History of inference timing and offset data
        """
        if not self.logger or len(inference_history) < 3:
            return
            
        self.logger.info("=== PATTERN DETECTION ===")
        
        # Check if offsets are consistent
        recent_offsets = [h['calculated_offset'] for h in inference_history[-5:]]
        if len(set(recent_offsets)) <= 2:
            self.logger.info(f"PATTERN: Offsets are quite consistent: {recent_offsets}")
        else:
            self.logger.info(f"PATTERN: Offsets vary: {recent_offsets}")
        
        # Check inference timing patterns
        inference_times = [h['inference_time'] for h in inference_history[-5:]]
        avg_inference_time = sum(inference_times) / len(inference_times)
        self.logger.info(f"PATTERN: Avg inference time: {avg_inference_time*1000:.1f}ms")
        
        # Check action execution rate during inference
        action_rates = []
        for h in inference_history[-3:]:
            actions_during = h['action_count_when_completed'] - h['inference_start_action']
            time_taken = h['inference_time']
            rate = actions_during / time_taken if time_taken > 0 else 0
            action_rates.append(rate)
        
        if action_rates:
            avg_rate = sum(action_rates) / len(action_rates)
            self.logger.info(f"PATTERN: Actions executed during inference: {avg_rate:.1f} actions/sec")
            
            # Check if this matches our 10Hz expectation
            expected_rate = 10.0  # 10Hz action timer
            if abs(avg_rate - expected_rate) > 2.0:
                self.logger.warning(
                    f"PATTERN ANOMALY: Action rate ({avg_rate:.1f}) differs significantly from expected 10Hz"
                )
        
        # Check for potential timing issues
        for i, h in enumerate(inference_history[-3:], start=len(inference_history)-3):
            start_action = h['inference_start_action']
            completion_action = h['action_count_when_completed']
            inference_time_ms = h['inference_time'] * 1000
            
            # Calculate theoretical actions that should have executed
            theoretical_actions = int(inference_time_ms / 100)  # 100ms per action at 10Hz
            actual_actions = completion_action - start_action
            
            self.logger.info(
                f"Chunk {i}: Inference took {inference_time_ms:.0f}ms, "
                f"theoretical actions: {theoretical_actions}, "
                f"actual actions: {actual_actions}, "
                f"difference: {actual_actions - theoretical_actions}"
            )
        
        self.logger.info("=== END PATTERN DETECTION ===")

    def create_comprehensive_analysis(self, raw_action_chunks, action_history, inference_history, chunk_visualization_data):
        """
        Create comprehensive analysis including both plots and text analysis.
        
        Args:
            raw_action_chunks: List of raw action chunk data with metadata
            action_history: History of executed actions
            inference_history: History of inference timing and offset data
            chunk_visualization_data: Additional visualization data
            
        Returns:
            dict: Dictionary containing paths to generated plots and analysis results
        """
        results = {
            'curves_plot': None,
            'analysis_plot': None,
            'success': False
        }
        
        try:
            # Generate action chunk curves plot
            curves_plot_path = self.plot_action_chunk_curves(raw_action_chunks, action_history, inference_history)
            if curves_plot_path:
                results['curves_plot'] = curves_plot_path
            
            # Generate analysis plot
            analysis_plot_path = self.plot_action_chunk_analysis(inference_history, action_history, chunk_visualization_data)
            if analysis_plot_path:
                results['analysis_plot'] = analysis_plot_path
            
            # Print text analysis
            self.print_chunk_curve_analysis(raw_action_chunks)
            self.print_offset_analysis(inference_history)
            self.detect_offset_patterns(inference_history)
            
            results['success'] = True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f'Error creating comprehensive analysis: {str(e)}')
                self.logger.error(f'Traceback: {traceback.format_exc()}')
        
        return results
