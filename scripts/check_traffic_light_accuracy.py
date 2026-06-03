#!/usr/bin/env python3
"""
File: check_traffic_light_accuracy.py
Project: Road Sign and Traffic Light Detection System
Author: Manish Ram

Purpose:
    Runs the detector on labeled traffic light images and compares the output
    label with the expected label from the file name.
"""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class ImageResult:
    image_path: Path
    expected_label: str
    predicted_labels: list[str]

    @property
    def passed(self) -> bool:
        return self.expected_label in self.predicted_labels


def expected_label_for(image_path: Path) -> str:
    name = image_path.name.upper()
    if name.startswith("RED_TRAFIC_LIGHT"):
        return "RED_TRAFFIC_LIGHT"
    if name.startswith("YELLOW_TRAFIC_LIGHT"):
        return "YELLOW_TRAFFIC_LIGHT"
    if name.startswith("GREEN_TRAFIC_LIGHT"):
        return "GREEN_TRAFFIC_LIGHT"
    raise ValueError(f"Cannot infer expected label from {image_path.name}")


def find_labeled_images(data_dir: Path) -> list[Path]:
    images = sorted(data_dir.glob("*_TRAFIC_LIGHT_*.png"))
    return [image for image in images if image.name.upper().startswith(("RED_", "YELLOW_", "GREEN_"))]


def read_predicted_labels(detector_path: Path, image_path: Path) -> list[str]:
    completed = subprocess.run(
        [str(detector_path), str(image_path)],
        check=True,
        capture_output=True,
        text=True,
    )

    labels: list[str] = []
    for line in completed.stdout.splitlines():
        if not line or line.startswith("type") or line.startswith("No detections"):
            continue

        label = line.split()[0]
        if label.endswith("_SIGN") or label.endswith("_LIGHT"):
            labels.append(label)

    return labels


def run_check(detector_path: Path, data_dir: Path) -> list[ImageResult]:
    results: list[ImageResult] = []
    for image_path in find_labeled_images(data_dir):
        expected_label = expected_label_for(image_path)
        predicted_labels = read_predicted_labels(detector_path, image_path)
        results.append(ImageResult(image_path, expected_label, predicted_labels))
    return results


def print_report(results: list[ImageResult]) -> None:
    if not results:
        print("No labeled traffic light images found.")
        return

    correct = sum(1 for result in results if result.passed)
    total = len(results)

    print("Traffic light accuracy report")
    print()
    print(f"{'image':35} {'expected':22} {'result':8} predictions")
    print("-" * 95)

    by_label: dict[str, list[ImageResult]] = {}
    for result in results:
        by_label.setdefault(result.expected_label, []).append(result)
        status = "PASS" if result.passed else "FAIL"
        predictions = ", ".join(result.predicted_labels) if result.predicted_labels else "none"
        print(f"{result.image_path.name:35} {result.expected_label:22} {status:8} {predictions}")

    print()
    print(f"Overall accuracy: {correct}/{total} = {(correct / total) * 100:.1f}%")

    for label in sorted(by_label):
        label_results = by_label[label]
        label_correct = sum(1 for result in label_results if result.passed)
        label_total = len(label_results)
        print(f"{label}: {label_correct}/{label_total} = {(label_correct / label_total) * 100:.1f}%")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check traffic light detection accuracy.")
    parser.add_argument(
        "--detector",
        type=Path,
        default=Path("build/RoadSignDetector"),
        help="Path to the built RoadSignDetector executable.",
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=Path("data"),
        help="Directory containing labeled traffic light images.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    detector_path = args.detector
    data_dir = args.data

    if not detector_path.exists():
        print(f"Detector executable was not found: {detector_path}")
        print("Build first with ./scripts/build_macos.sh")
        return 1

    if not data_dir.exists():
        print(f"Data directory was not found: {data_dir}")
        return 1

    results = run_check(detector_path, data_dir)
    print_report(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
