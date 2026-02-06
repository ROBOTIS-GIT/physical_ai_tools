#!/usr/bin/env python3
"""
LeRobot Dataset Instruction Manager

데이터셋의 task instruction을 편집하는 스크립트입니다.

Usage:
    python manage_instructions.py /path/to/dataset

Features:
    - 3가지 편집 모드 지원 (전체/선택/일괄)
    - LeRobot 표준 포맷 준수 (episodes.jsonl에 STRING 배열 사용)
    - 자동 백업 생성
    - task 문자열 자동 중복 제거
"""

import json
import shutil
from datetime import datetime
from pathlib import Path
import sys


class InstructionManager:
    """LeRobot 데이터셋의 task instruction 관리 클래스"""

    def __init__(self):
        self.tasks = {}  # {task_index: task_string}
        self.episodes = []  # [{episode_index, tasks, length}, ...]

    def load_tasks(self, dataset_dir: Path) -> dict:
        """tasks.jsonl 로드"""
        tasks_path = dataset_dir / "meta" / "tasks.jsonl"
        tasks = {}
        if tasks_path.exists():
            with open(tasks_path, 'r', encoding='utf-8') as f:
                for line in f:
                    if line.strip():
                        data = json.loads(line)
                        tasks[data['task_index']] = data['task']
        return tasks

    def load_episodes(self, dataset_dir: Path) -> list:
        """episodes.jsonl 로드"""
        episodes_path = dataset_dir / "meta" / "episodes.jsonl"
        episodes = []
        if episodes_path.exists():
            with open(episodes_path, 'r', encoding='utf-8') as f:
                for line in f:
                    if line.strip():
                        data = json.loads(line)
                        episodes.append(data)
        return episodes

    def backup_files(self, dataset_dir: Path):
        """meta 폴더 백업"""
        meta_dir = dataset_dir / "meta"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = dataset_dir / f"meta_backup_{timestamp}"

        if meta_dir.exists():
            shutil.copytree(meta_dir, backup_dir)
            print(f"  Backup created: {backup_dir}")

    def write_tasks(self, dataset_dir: Path, tasks: dict):
        """tasks.jsonl 저장"""
        tasks_path = dataset_dir / "meta" / "tasks.jsonl"
        with open(tasks_path, 'w', encoding='utf-8') as f:
            for task_index in sorted(tasks.keys()):
                data = {"task_index": task_index, "task": tasks[task_index]}
                f.write(json.dumps(data, ensure_ascii=False) + '\n')

    def write_episodes(self, dataset_dir: Path, episodes: list):
        """episodes.jsonl 저장 (tasks는 STRING 배열로 저장)"""
        episodes_path = dataset_dir / "meta" / "episodes.jsonl"
        with open(episodes_path, 'w', encoding='utf-8') as f:
            for ep in episodes:
                data = {
                    "episode_index": ep["episode_index"],
                    "tasks": ep["tasks"],  # STRING 배열
                    "length": ep["length"]
                }
                f.write(json.dumps(data, ensure_ascii=False) + '\n')

    def _get_mode_selection(self) -> str | None:
        """편집 모드 선택"""
        print("\n=== Edit Mode ===")
        print("  [1] 전체 편집 - Edit all episodes one by one")
        print("  [2] 선택 편집 - Select specific episodes to edit")
        print("  [3] 일괄 적용 - Apply same instruction to multiple episodes")
        print("  [q] 취소")

        while True:
            mode = input("\nChoose mode [1-3] or 'q': ").strip()
            if mode == 'q':
                return None
            if mode in ['1', '2', '3']:
                return mode
            print("Invalid choice. Please enter 1, 2, 3, or q.")

    def _get_episode_selection(self, mode: str, total_episodes: int) -> list | None:
        """에피소드 선택"""
        if mode == '1':
            return list(range(total_episodes))

        while True:
            print(f"\nTotal episodes: {total_episodes} (indices: 0-{total_episodes-1})")
            selection = input(
                "Enter episode numbers (comma-separated, e.g., '0,3,5,7')\n"
                "or 'all' for all episodes, 'q' to cancel: "
            ).strip()

            if selection.lower() == 'q':
                return None

            if selection.lower() == 'all':
                return list(range(total_episodes))

            try:
                indices = [int(x.strip()) for x in selection.split(',')]
                invalid = [idx for idx in indices if idx < 0 or idx >= total_episodes]
                if invalid:
                    print(f"  Invalid episode indices: {invalid}")
                    print(f"  Valid range: 0-{total_episodes-1}")
                    continue

                selected = sorted(set(indices))
                print(f"  Selected {len(selected)} episodes: {selected}")
                return selected

            except ValueError:
                print("  Invalid format. Please enter numbers separated by commas.")
                continue

    def _get_bulk_instruction(self) -> str | None:
        """일괄 적용할 instruction 입력"""
        while True:
            instruction = input("\nEnter instruction to apply to all selected episodes: ").strip()

            if instruction.lower() == 'q':
                return None

            if not instruction:
                print("  Instruction cannot be empty")
                continue

            if len(instruction) > 500:
                print(f"  Warning: Instruction is very long ({len(instruction)} chars)")
                confirm = input("  Continue? [y/n]: ").strip().lower()
                if confirm != 'y':
                    continue

            return instruction

    def _review_changes(self, episodes: list, changes: dict, selected: list) -> bool:
        """변경사항 검토"""
        print("\n" + "=" * 50)
        print("=== Review Changes ===")
        print("=" * 50)
        print(f"Total episodes in dataset: {len(episodes)}")
        print(f"Selected episodes: {len(selected)}")
        print(f"Modified: {len(changes)}\n")

        if changes:
            print("Changes to be applied:")
            for ep_idx in sorted(changes.keys()):
                old_task = episodes[ep_idx].get("tasks", ["N/A"])[0]
                new_task = changes[ep_idx]
                if old_task != new_task:
                    print(f"  Episode {ep_idx}:")
                    print(f"    Before: \"{old_task}\"")
                    print(f"    After:  \"{new_task}\"")

        print()
        confirm = input("Apply changes? [y/n]: ").strip().lower()
        return confirm == 'y'

    def _apply_changes(self, dataset_dir: Path, episodes: list, changes: dict):
        """변경사항 적용"""
        # 모든 unique task string 수집
        all_task_strings = set()

        for ep in episodes:
            ep_idx = ep["episode_index"]
            if ep_idx in changes:
                all_task_strings.add(changes[ep_idx])
            else:
                all_task_strings.update(ep.get("tasks", []))

        # 새 tasks.jsonl 생성 (정렬된 순서로)
        new_tasks = {}
        for i, task_str in enumerate(sorted(all_task_strings)):
            new_tasks[i] = task_str

        # 새 episodes.jsonl 생성
        new_episodes = []
        for ep in episodes:
            ep_idx = ep["episode_index"]
            new_ep = {
                "episode_index": ep_idx,
                "tasks": [changes[ep_idx]] if ep_idx in changes else ep.get("tasks", []),
                "length": ep["length"]
            }
            new_episodes.append(new_ep)

        # 백업 후 저장
        self.backup_files(dataset_dir)
        self.write_tasks(dataset_dir, new_tasks)
        self.write_episodes(dataset_dir, new_episodes)

        print(f"\n  Saved {len(new_tasks)} tasks to tasks.jsonl")
        print(f"  Saved {len(new_episodes)} episodes to episodes.jsonl")

    def show_current_state(self, dataset_dir: Path):
        """현재 데이터셋 상태 표시"""
        tasks = self.load_tasks(dataset_dir)
        episodes = self.load_episodes(dataset_dir)

        print("\n" + "=" * 60)
        print(f"Dataset: {dataset_dir}")
        print("=" * 60)

        print(f"\nTasks ({len(tasks)}):")
        for idx, task in tasks.items():
            print(f"  [{idx}] {task}")

        print(f"\nEpisodes ({len(episodes)}):")
        for ep in episodes[:10]:  # 처음 10개만 표시
            tasks_str = ", ".join(ep.get("tasks", ["N/A"]))
            print(f"  Episode {ep['episode_index']}: \"{tasks_str}\" ({ep['length']} frames)")

        if len(episodes) > 10:
            print(f"  ... and {len(episodes) - 10} more episodes")

    def interactive_edit(self, dataset_dir: Path):
        """대화형 편집"""
        # 현재 상태 로드
        tasks = self.load_tasks(dataset_dir)
        episodes = self.load_episodes(dataset_dir)
        total_episodes = len(episodes)

        if total_episodes == 0:
            print("No episodes found in dataset")
            return

        # 현재 상태 표시
        self.show_current_state(dataset_dir)

        # 모드 선택
        mode = self._get_mode_selection()
        if mode is None:
            print("Cancelled")
            return

        # 에피소드 선택
        selected_indices = self._get_episode_selection(mode, total_episodes)
        if selected_indices is None:
            print("Cancelled")
            return

        # instruction 수집
        episode_task_changes = {}  # ep_idx → new_task_string

        if mode == '3':
            # 일괄 적용
            new_task = self._get_bulk_instruction()
            if new_task is None:
                print("Cancelled")
                return
            for ep_idx in selected_indices:
                episode_task_changes[ep_idx] = new_task
        else:
            # 개별 편집 (모드 1, 2)
            print("\n--- Individual Editing ---")
            print("(Press Enter to keep current, 'q' to cancel)\n")

            for ep_idx in selected_indices:
                ep = episodes[ep_idx]
                current_tasks = ep.get("tasks", [])
                current_task = current_tasks[0] if current_tasks else "N/A"

                print(f"Episode {ep_idx} ({ep['length']} frames)")
                print(f"  Current: \"{current_task}\"")

                new_task = input("  New instruction: ").strip()

                if new_task.lower() == 'q':
                    print("Cancelled")
                    return

                if new_task and new_task != current_task:
                    episode_task_changes[ep_idx] = new_task
                    print(f"  -> Changed to: \"{new_task}\"")
                else:
                    print("  -> Kept current")
                print()

        # 변경사항이 없으면 종료
        if not episode_task_changes:
            print("No changes to apply")
            return

        # 변경사항 검토
        if not self._review_changes(episodes, episode_task_changes, selected_indices):
            print("Cancelled")
            return

        # 적용
        self._apply_changes(dataset_dir, episodes, episode_task_changes)
        print("\nDone!")


def main():
    if len(sys.argv) < 2:
        print("Usage: python manage_instructions.py <dataset_path>")
        print("\nExample:")
        print("  python manage_instructions.py /path/to/my_dataset")
        sys.exit(1)

    dataset_path = Path(sys.argv[1])

    if not dataset_path.exists():
        print(f"Error: Dataset path not found: {dataset_path}")
        sys.exit(1)

    meta_path = dataset_path / "meta"
    if not meta_path.exists():
        print(f"Error: No 'meta' folder found in: {dataset_path}")
        sys.exit(1)

    manager = InstructionManager()
    manager.interactive_edit(dataset_path)


if __name__ == "__main__":
    main()
