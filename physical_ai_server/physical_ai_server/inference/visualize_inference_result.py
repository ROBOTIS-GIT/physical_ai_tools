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
import threading
import time


class InferenceResultVisualizer:

    def __init__(self, logger=None):
        self.logger = logger
        self.plot_thread = None
        self.plot_lock = threading.Lock()
        self.is_plotting = False

    def plot_action_chunk_curves(self, raw_action_chunks, action_history, inference_history):
        try:
            # Set matplotlib to use Agg backend for thread safety
            import matplotlib
            matplotlib.use('Agg')
            
            joint_count = raw_action_chunks[0]['joint_count']

            subplot_height = 3.5
            total_height = max(subplot_height * joint_count + 2, 10)  # Minimum 10 inches

            fig, axes = plt.subplots(joint_count, 1, figsize=(16, total_height), sharex=True)
            if joint_count == 1:
                axes = [axes]

            fig.suptitle(
                f'Comprehensive View ({len(raw_action_chunks)} chunks)\n' +
                'Outputs vs Actual (○: Inference Request, ×: Inference Complete)',
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

                    colors = [
                        'red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray',
                        'olive', 'cyan', 'magenta', 'yellow', 'navy', 'lime', 'teal', 'maroon'
                    ]

                    for chunk_idx, chunk_data in enumerate(raw_action_chunks):
                        inference_start_step = chunk_data['inference_start_step']
                        raw_actions = chunk_data['raw_actions']

                        if not raw_actions or len(raw_actions[0]) <= joint_idx:
                            continue

                        applied_offset = 0
                        used_chunk_size = len(raw_actions)
                        if chunk_idx < len(inference_history):
                            applied_offset = inference_history[chunk_idx]['calculated_offset']
                            used_chunk_size = inference_history[chunk_idx].get('used_chunk_size', len(raw_actions))

                        if applied_offset < len(raw_actions):
                            used_actions = raw_actions[applied_offset:applied_offset + used_chunk_size]
                            joint_values = [action[joint_idx] for action in used_actions]

                            # Create step numbers for the actually used actions
                            actual_start_step = inference_start_step + applied_offset
                            chunk_steps = list(
                                range(actual_start_step, actual_start_step + len(joint_values))
                            )

                            color = colors[chunk_idx % len(colors)]
                            ax.plot(chunk_steps, joint_values, color=color, linestyle='--', 
                                   linewidth=1.5, alpha=0.7, 
                                   label=f'Inference {chunk_idx+1} (used: {len(joint_values)}/{len(raw_actions)} actions)')

                            # Mark the start point (when inference was requested)
                            ax.scatter(
                                [inference_start_step], [raw_actions[0][joint_idx]], 
                                color=color, s=50, marker='o', alpha=0.8,
                                label=f'Inf{chunk_idx+1} Request' if chunk_idx < 3 else None)

                            # Mark the actual usage start point (after offset)
                            if applied_offset > 0:
                                ax.scatter(
                                    [actual_start_step], [joint_values[0]], 
                                    color=color, s=50, marker='>', alpha=0.8,
                                    label=f'Inf{chunk_idx+1} Used Start' if chunk_idx < 3 else None)

                            # Mark completion point (when this chunk was processed)
                            if chunk_idx < len(inference_history):
                                completion_action = inference_history[chunk_idx][
                                    'action_count_when_completed']
                                ax.scatter(
                                    [completion_action],
                                    [raw_actions[
                                        min(applied_offset, len(raw_actions)-1)][joint_idx]], 
                                    color=color, s=50, marker='x', alpha=0.8,
                                    label=f'Inf{chunk_idx+1} Complete' if chunk_idx < 3 else None)

                    ax.set_ylabel(f'Joint {joint_idx} Value')
                    ax.grid(True, alpha=0.3)

                    # Limit legend entries to avoid overcrowding
                    handles, labels = ax.get_legend_handles_labels()
                    if len(handles) > 12:  # Limit to 12 entries for 10 chunks
                        handles = handles[:12]
                        labels = labels[:12]
                        labels[-1] = f'... and {len(handles) - 11} more'

                    ax.legend(
                        handles, labels, bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)

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

    def process_visualization_data(
            self, action_chunk, inference_time, worker_start_count,
            inference_history, raw_action_chunks):

        current_chunk_data = {
            'chunk_id': len(inference_history),
            'inference_start_action': worker_start_count,
            'action_count_when_completed': 0,  # Will be updated later
            'calculated_offset': 0,  # Will be calculated later
            'original_chunk_size': len(action_chunk),
            'inference_time': inference_time,
            'timestamp': time.time()
        }
        inference_history.append(current_chunk_data)

        # Convert to standard list format for consistency
        if hasattr(action_chunk, 'tolist'):
            action_chunk_list = action_chunk.tolist()
        else:
            action_chunk_list = list(action_chunk) if action_chunk is not None else []

        # Get first action for joint count calculation
        first_action = action_chunk_list[0] if action_chunk_list else None

        raw_chunk_data = {
            'chunk_id': len(raw_action_chunks),
            'inference_start_step': worker_start_count,
            'raw_actions': action_chunk_list,
            'joint_count': len(first_action) if first_action else 0,
            'timestamp': time.time()
        }
        raw_action_chunks.append(raw_chunk_data)

    def process_complete_inference_visualization(
            self,
            action_chunk,
            inference_time, worker_start_count,
            offset_action_chunk, used_action_count, actions_executed_during_inference,
            inference_history, raw_action_chunks, action_history,
            chunk_visualization_data, logger=None):

        self.process_visualization_data(
            action_chunk, inference_time, worker_start_count,
            inference_history, raw_action_chunks
        )

        self.update_visualization_data_with_final_values(
            inference_history, used_action_count, actions_executed_during_inference
        )

        self.update_final_visualization_data(
            offset_action_chunk, used_action_count, worker_start_count,
            actions_executed_during_inference, len(action_chunk),
            inference_history, chunk_visualization_data
        )

        self.generate_visualization_output(
            inference_history, raw_action_chunks,
            action_history, chunk_visualization_data, logger
        )

    def update_visualization_data_with_final_values(
            self, inference_history, used_action_count,
            actions_executed_during_inference):

        if inference_history:
            current_chunk_data = inference_history[-1]
            current_chunk_data['action_count_when_completed'] = used_action_count
            current_chunk_data['calculated_offset'] = actions_executed_during_inference

    def update_final_visualization_data(
                self,
                offset_action_chunk,
                used_action_count,
                worker_start_count,
                actions_executed_during_inference,
                original_chunk_size,
                inference_history,
                chunk_visualization_data
            ):

        if inference_history:
            current_chunk_data = inference_history[-1]
            current_chunk_data['used_chunk_size'] = len(offset_action_chunk)
            current_chunk_data['action_start_from'] = used_action_count + 1

        chunk_visualization_data['chunk_start_actions'].append(used_action_count + 1)
        chunk_visualization_data['chunk_inference_starts'].append(worker_start_count)
        chunk_visualization_data['chunk_offsets'].append(actions_executed_during_inference)
        chunk_visualization_data['chunk_sizes'].append(original_chunk_size)
        chunk_visualization_data['chunk_used_sizes'].append(len(offset_action_chunk))

    def generate_visualization_output(
                self,
                inference_history,
                raw_action_chunks,
                action_history,
                chunk_visualization_data,
                logger=None
            ):
        if len(inference_history) % 20 == 0:
            if logger:
                logger.info(
                    f'Starting comprehensive chunk analysis graph {len(inference_history)})')

            # Start plot generation in background thread
            self._start_plot_generation_thread(
                raw_action_chunks,
                list(action_history),
                inference_history,
                chunk_visualization_data
            )

    def _start_plot_generation_thread(
                self,
                raw_action_chunks,
                action_history,
                inference_history,
                chunk_visualization_data
            ):
        with self.plot_lock:
            if self.is_plotting:
                if self.logger:
                    self.logger.info(
                        '📊 Plot generation already in progress, skipping this request')
                return

            self.is_plotting = True

        # Create deep copies of data to avoid race conditions
        try:
            import copy
            raw_action_chunks_copy = copy.deepcopy(raw_action_chunks)
            action_history_copy = copy.deepcopy(action_history)
            inference_history_copy = copy.deepcopy(inference_history)
            chunk_visualization_data_copy = copy.deepcopy(chunk_visualization_data)

            # Start background thread
            self.plot_thread = threading.Thread(
                target=self._generate_plot_in_background,
                args=(
                    raw_action_chunks_copy,
                    action_history_copy,
                    inference_history_copy,
                    chunk_visualization_data_copy
                ),
                daemon=True  # Thread will be killed when main program exits
            )
            self.plot_thread.start()

            if self.logger:
                self.logger.info('Plot generation thread started successfully')

        except Exception as e:
            with self.plot_lock:
                self.is_plotting = False
            if self.logger:
                self.logger.error(f'Failed to start plot generation thread: {str(e)}')

    def _generate_plot_in_background(
                self,
                raw_action_chunks,
                action_history,
                inference_history,
                chunk_visualization_data
            ):
        try:
            start_time = time.time()
            if self.logger:
                self.logger.info('Background plot generation started')

            results = self.create_comprehensive_analysis(
                raw_action_chunks,
                action_history,
                inference_history,
                chunk_visualization_data
            )

            elapsed_time = time.time() - start_time
            if self.logger:
                if results['success']:
                    self.logger.info(
                        f'Background plot generation completed successfully in {elapsed_time:.2f}s')
                    if results['curves_plot']:
                        self.logger.info(
                            f'Plot saved to: {results['curves_plot']}')
                else:
                    self.logger.error(
                        f'Background plot generation failed after {elapsed_time:.2f}s')

        except Exception as e:
            if self.logger:
                self.logger.error(
                    f'Error in background plot generation: {str(e)}')
                self.logger.error(
                    f'Traceback: {traceback.format_exc()}')
        finally:
            # Always reset the plotting flag
            with self.plot_lock:
                self.is_plotting = False

    def is_plot_generation_active(self):
        with self.plot_lock:
            return self.is_plotting

    def wait_for_plot_completion(self, timeout=30.0):
        if self.plot_thread and self.plot_thread.is_alive():
            if self.logger:
                self.logger.info(
                    f'Waiting for plot generation to complete (timeout: {timeout}s)')
            self.plot_thread.join(timeout=timeout)

            if self.plot_thread.is_alive():
                if self.logger:
                    self.logger.warning(
                        'Plot generation thread did not complete within timeout')
                return False
            else:
                if self.logger:
                    self.logger.info(
                        'Plot generation completed')
                return True
        return True

    def create_comprehensive_analysis(
                self,
                raw_action_chunks,
                action_history,
                inference_history,
                chunk_visualization_data
            ):

        results = {
            'curves_plot': None,
            'success': False
        }

        try:
            # Generate action chunk curves plot
            curves_plot_path = self.plot_action_chunk_curves(
                raw_action_chunks,
                action_history,
                inference_history
            )
            if curves_plot_path:
                results['curves_plot'] = curves_plot_path

            results['success'] = True

        except Exception as e:
            if self.logger:
                self.logger.error(
                    f'Error creating comprehensive analysis: {str(e)}'
                )
                self.logger.error(
                    f'Traceback: {traceback.format_exc()}'
                )

        return results
