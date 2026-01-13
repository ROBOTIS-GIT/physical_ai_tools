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
# Author: AI Assistant (Sisyphus)

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from physical_ai_server.data_processing.lerobot_dataset_validator import (
    LeRobotDatasetValidator,
    ValidationResult,
    ValidationSeverity,
)


def print_validation_result(result: ValidationResult, verbose: bool = False):
    print("\n" + "=" * 70)
    print("LeRobot Dataset Validation Report")
    print("=" * 70)

    status_icon = "✓" if result.is_valid else "✗"
    status_text = "VALID" if result.is_valid else "INVALID"
    print(f"\nStatus: {status_icon} {status_text}")
    print(f"Dataset: {result.dataset_path}")
    print(f"Errors: {result.error_count}")
    print(f"Warnings: {result.warning_count}")

    if result.summary:
        print("\n" + "-" * 70)
        print("Summary:")
        print("-" * 70)
        for key, value in result.summary.items():
            if key not in ["dataset_path", "error_count", "warning_count"]:
                if isinstance(value, list) and len(value) > 5:
                    print(
                        f"  {key}: [{', '.join(str(v) for v in value[:5])}, ... ({len(value)} total)]"
                    )
                else:
                    print(f"  {key}: {value}")

    if result.issues:
        print("\n" + "-" * 70)
        print("Issues:")
        print("-" * 70)

        errors = [i for i in result.issues if i.severity == ValidationSeverity.ERROR]
        warnings = [
            i for i in result.issues if i.severity == ValidationSeverity.WARNING
        ]

        if errors:
            print(f"\n✗ Errors ({len(errors)}):")
            for issue in errors:
                print(f"  [{issue.category}] {issue.message}")
                if verbose and issue.details:
                    for k, v in issue.details.items():
                        print(f"    {k}: {v}")

        if warnings:
            print(f"\n⚠ Warnings ({len(warnings)}):")
            for issue in warnings:
                print(f"  [{issue.category}] {issue.message}")
                if verbose and issue.details:
                    for k, v in issue.details.items():
                        print(f"    {k}: {v}")

    print("\n" + "=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="Validate LeRobot v2.1 dataset format and data integrity",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /path/to/dataset
  %(prog)s /path/to/dataset --verbose
  %(prog)s /path/to/dataset --json --output report.json

Validation Checks:
  - Directory structure (meta/, data/, videos/)
  - Required files (info.json, episodes.jsonl, tasks.jsonl)
  - info.json schema and required fields
  - Parquet file columns and data types
  - Data integrity (NaN, Inf, empty values, all-zeros)
  - Video-Parquet frame count synchronization
  - Cross-file consistency (episode/task counts)
        """,
    )

    parser.add_argument(
        "dataset_path",
        type=str,
        help="Path to the LeRobot dataset directory",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Show detailed information for each issue",
    )

    parser.add_argument(
        "--json",
        action="store_true",
        help="Output result as JSON",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Write validation report to file (JSON format)",
    )

    args = parser.parse_args()

    dataset_path = Path(args.dataset_path)

    if not dataset_path.exists():
        print(f"Error: Dataset path does not exist: {dataset_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Validating dataset: {dataset_path}")
    print("Please wait...")

    validator = LeRobotDatasetValidator(dataset_path)
    result = validator.validate()

    if args.json:
        print(result.to_json())
    else:
        print_validation_result(result, verbose=args.verbose)

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(result.to_json())
        print(f"\nReport saved to: {output_path}")

    sys.exit(0 if result.is_valid else 1)


if __name__ == "__main__":
    main()
