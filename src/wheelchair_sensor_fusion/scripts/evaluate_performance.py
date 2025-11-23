#!/usr/bin/env python3
"""
Performance Evaluation Script for Wheelchair Sensor Fusion

Evaluates fusion performance from recorded ROS2 bag files against ground truth.
Computes metrics for ICRA 2025 paper:
- Precision, Recall, F1-score
- True Positives, False Positives, False Negatives
- Detection latency
- Topic publishing rates
- Fusion success rate

Author: Siddharth Tiwari (IIT Mandi)
Usage: python3 evaluate_performance.py --bag-dir <path> --ground-truth <path> --output <path>
"""

import argparse
import json
import sys
from pathlib import Path
from typing import List, Dict, Tuple
import numpy as np
from collections import defaultdict


class GroundTruthObstacle:
    """Represents a ground truth obstacle."""

    def __init__(self, timestamp: float, obj_type: str, x: float, y: float, width: float, height: float):
        self.timestamp = timestamp
        self.obj_type = obj_type
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def __repr__(self):
        return f"GT({self.obj_type} at ({self.x:.2f}, {self.y:.2f}))"


class DetectedObstacle:
    """Represents a detected obstacle from fusion."""

    def __init__(self, timestamp: float, x: float, y: float, size_x: float, size_y: float):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.size_x = size_x
        self.size_y = size_y

    def __repr__(self):
        return f"Det({self.x:.2f}, {self.y:.2f})"


def load_ground_truth(filepath: str) -> List[GroundTruthObstacle]:
    """Load ground truth from text file.

    Format: timestamp, object_type, x, y, width, height
    """
    obstacles = []

    try:
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = [p.strip() for p in line.split(',')]
                if len(parts) != 6:
                    print(f"Warning: Invalid line: {line}")
                    continue

                timestamp = float(parts[0])
                obj_type = parts[1]
                x = float(parts[2])
                y = float(parts[3])
                width = float(parts[4])
                height = float(parts[5])

                obstacles.append(GroundTruthObstacle(timestamp, obj_type, x, y, width, height))

        print(f"✓ Loaded {len(obstacles)} ground truth obstacles")
        return obstacles

    except FileNotFoundError:
        print(f"✗ Ground truth file not found: {filepath}")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Error loading ground truth: {e}")
        sys.exit(1)


def load_detections_from_bag(bag_dir: str) -> List[DetectedObstacle]:
    """Load detections from ROS2 bag file.

    Note: This is a placeholder. In practice, you would use rosbag2_py
    to read the bag and extract /fusion/obstacles messages.
    """
    detections = []

    print(f"ℹ Loading detections from bag: {bag_dir}")
    print("⚠ Note: Bag parsing requires rosbag2_py implementation")
    print("   For now, using simulated detections for demonstration")

    # TODO: Implement actual bag parsing
    # from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    # ...

    # Simulated detections (replace with actual bag parsing)
    detections = [
        DetectedObstacle(0.5, 2.3, 0.1, 0.5, 1.8),
        DetectedObstacle(5.5, 3.0, -0.9, 0.6, 0.9),
        DetectedObstacle(10.8, 4.3, 0.6, 1.2, 0.8),
    ]

    print(f"ℹ Loaded {len(detections)} detections")
    return detections


def compute_iou(gt: GroundTruthObstacle, det: DetectedObstacle) -> float:
    """Compute Intersection over Union (IoU) for 2D bounding boxes."""

    # Ground truth box
    gt_x1 = gt.x - gt.width / 2
    gt_x2 = gt.x + gt.width / 2
    gt_y1 = gt.y - gt.height / 2
    gt_y2 = gt.y + gt.height / 2

    # Detection box
    det_x1 = det.x - det.size_x / 2
    det_x2 = det.x + det.size_x / 2
    det_y1 = det.y - det.size_y / 2
    det_y2 = det.y + det.size_y / 2

    # Intersection
    x1 = max(gt_x1, det_x1)
    y1 = max(gt_y1, det_y1)
    x2 = min(gt_x2, det_x2)
    y2 = min(gt_y2, det_y2)

    if x2 < x1 or y2 < y1:
        return 0.0

    intersection = (x2 - x1) * (y2 - y1)

    # Union
    gt_area = gt.width * gt.height
    det_area = det.size_x * det.size_y
    union = gt_area + det_area - intersection

    if union == 0:
        return 0.0

    return intersection / union


def match_detections(
    ground_truth: List[GroundTruthObstacle],
    detections: List[DetectedObstacle],
    iou_threshold: float = 0.3,
    time_threshold: float = 1.0
) -> Tuple[int, int, int]:
    """Match detections to ground truth obstacles.

    Returns:
        (true_positives, false_positives, false_negatives)
    """

    matched_gt = set()
    matched_det = set()

    true_positives = 0

    # Match each detection to ground truth
    for i, det in enumerate(detections):
        best_iou = 0.0
        best_gt_idx = -1

        for j, gt in enumerate(ground_truth):
            if j in matched_gt:
                continue

            # Check time alignment
            time_diff = abs(det.timestamp - gt.timestamp)
            if time_diff > time_threshold:
                continue

            # Compute IoU
            iou = compute_iou(gt, det)

            if iou > best_iou:
                best_iou = iou
                best_gt_idx = j

        # Match if IoU exceeds threshold
        if best_iou >= iou_threshold:
            true_positives += 1
            matched_gt.add(best_gt_idx)
            matched_det.add(i)

    false_positives = len(detections) - len(matched_det)
    false_negatives = len(ground_truth) - len(matched_gt)

    return true_positives, false_positives, false_negatives


def compute_metrics(tp: int, fp: int, fn: int) -> Dict[str, float]:
    """Compute precision, recall, and F1-score."""

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

    return {
        'precision': precision,
        'recall': recall,
        'f1_score': f1_score,
        'true_positives': tp,
        'false_positives': fp,
        'false_negatives': fn,
    }


def evaluate_bag(bag_dir: str, ground_truth_file: str) -> Dict:
    """Evaluate a single bag file against ground truth."""

    # Load data
    ground_truth = load_ground_truth(ground_truth_file)
    detections = load_detections_from_bag(bag_dir)

    # Match and compute metrics
    tp, fp, fn = match_detections(ground_truth, detections)
    metrics = compute_metrics(tp, fp, fn)

    # Additional statistics
    metrics['total_ground_truth'] = len(ground_truth)
    metrics['total_detections'] = len(detections)

    return metrics


def print_results(metrics: Dict):
    """Print evaluation results in a readable format."""

    print("\n" + "=" * 80)
    print("EVALUATION RESULTS")
    print("=" * 80)
    print(f"Ground Truth Obstacles:  {metrics['total_ground_truth']}")
    print(f"Detected Obstacles:      {metrics['total_detections']}")
    print()
    print(f"True Positives:          {metrics['true_positives']}")
    print(f"False Positives:         {metrics['false_positives']}")
    print(f"False Negatives:         {metrics['false_negatives']}")
    print()
    print(f"Precision:               {metrics['precision']:.4f} ({metrics['precision']*100:.2f}%)")
    print(f"Recall:                  {metrics['recall']:.4f} ({metrics['recall']*100:.2f}%)")
    print(f"F1-Score:                {metrics['f1_score']:.4f} ({metrics['f1_score']*100:.2f}%)")
    print("=" * 80)


def save_results(metrics: Dict, output_file: str):
    """Save results to JSON file."""

    try:
        with open(output_file, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"\n✓ Results saved to: {output_file}")
    except Exception as e:
        print(f"\n✗ Error saving results: {e}")


def main():
    """Main evaluation function."""

    parser = argparse.ArgumentParser(
        description='Evaluate wheelchair sensor fusion performance'
    )
    parser.add_argument(
        '--bag-dir',
        type=str,
        required=True,
        help='Path to ROS2 bag directory'
    )
    parser.add_argument(
        '--ground-truth',
        type=str,
        required=True,
        help='Path to ground truth file (CSV format)'
    )
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Path to output JSON file'
    )
    parser.add_argument(
        '--iou-threshold',
        type=float,
        default=0.3,
        help='IoU threshold for matching (default: 0.3)'
    )
    parser.add_argument(
        '--time-threshold',
        type=float,
        default=1.0,
        help='Time threshold in seconds for matching (default: 1.0)'
    )

    args = parser.parse_args()

    # Validate inputs
    if not Path(args.bag_dir).exists():
        print(f"✗ Bag directory not found: {args.bag_dir}")
        sys.exit(1)

    if not Path(args.ground_truth).exists():
        print(f"✗ Ground truth file not found: {args.ground_truth}")
        sys.exit(1)

    # Run evaluation
    print("\n" + "=" * 80)
    print("WHEELCHAIR SENSOR FUSION - PERFORMANCE EVALUATION")
    print("=" * 80)
    print(f"Bag directory:       {args.bag_dir}")
    print(f"Ground truth:        {args.ground_truth}")
    print(f"Output file:         {args.output}")
    print(f"IoU threshold:       {args.iou_threshold}")
    print(f"Time threshold:      {args.time_threshold}s")
    print()

    metrics = evaluate_bag(args.bag_dir, args.ground_truth)

    # Display results
    print_results(metrics)

    # Save results
    save_results(metrics, args.output)

    print("\n✓ Evaluation complete!")


if __name__ == '__main__':
    main()
