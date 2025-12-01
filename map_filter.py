#!/usr/bin/env python3
"""
Map Noise Filter - Apply kernels to clean up SLAM maps

Applies various image processing techniques to remove noise from occupancy grid maps:
- Morphological operations (erosion, dilation, opening, closing)
- Median filtering
- Gaussian blur
- Custom kernels

Usage:
    python3 map_filter.py input_map.yaml [--output filtered_map] [--method all]

Methods:
    - median: Median filter (removes salt-and-pepper noise)
    - morpho: Morphological opening+closing (removes small artifacts)
    - gaussian: Gaussian blur (smooths edges)
    - all: Apply all methods and save each result
"""

import argparse
import os
import sys
import yaml
import numpy as np
from PIL import Image
import cv2
from pathlib import Path


def load_map(yaml_path: str) -> tuple:
    """Load map from YAML and PGM files"""
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    # Get PGM path (relative to YAML)
    yaml_dir = Path(yaml_path).parent
    pgm_path = yaml_dir / map_info['image']

    # Load image
    img = Image.open(pgm_path)
    map_array = np.array(img)

    print(f"Loaded map: {pgm_path}")
    print(f"  Size: {map_array.shape}")
    print(f"  Resolution: {map_info['resolution']} m/pixel")
    print(f"  Origin: {map_info['origin']}")

    return map_array, map_info, pgm_path


def save_map(map_array: np.ndarray, map_info: dict, output_base: str):
    """Save map to YAML and PGM files"""
    # Save PGM
    pgm_path = f"{output_base}.pgm"
    img = Image.fromarray(map_array.astype(np.uint8))
    img.save(pgm_path)

    # Update and save YAML
    yaml_path = f"{output_base}.yaml"
    map_info_copy = map_info.copy()
    map_info_copy['image'] = os.path.basename(pgm_path)

    with open(yaml_path, 'w') as f:
        yaml.dump(map_info_copy, f, default_flow_style=False)

    print(f"Saved: {yaml_path} and {pgm_path}")


def apply_median_filter(map_array: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Median filter - excellent for salt-and-pepper noise
    Preserves edges while removing isolated noise pixels
    """
    print(f"  Applying median filter (kernel={kernel_size})")

    # Separate known and unknown regions
    # In ROS maps: 205 = unknown, 254 = free, 0 = occupied
    unknown_mask = map_array == 205
    known = map_array.copy()

    # Apply median filter to known regions
    filtered = cv2.medianBlur(known, kernel_size)

    # Restore unknown regions
    filtered[unknown_mask] = 205

    return filtered


def apply_morphological(map_array: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Morphological opening + closing
    - Opening (erosion then dilation): removes small white noise
    - Closing (dilation then erosion): removes small black noise
    """
    print(f"  Applying morphological operations (kernel={kernel_size})")

    # Create kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    # Separate regions
    unknown_mask = map_array == 205

    # Convert to binary for morphological ops
    # Free space (254) -> 255, Occupied (0) -> 0
    binary = map_array.copy()
    binary[binary == 254] = 255
    binary[unknown_mask] = 128  # temp value for unknown

    # Opening removes small bright spots (noise in free space)
    opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # Closing removes small dark spots (noise in occupied space)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

    # Convert back to ROS map format
    result = closed.copy()
    result[result == 255] = 254  # free
    result[result == 128] = 205  # unknown
    result[unknown_mask] = 205  # restore unknown

    return result


def apply_gaussian(map_array: np.ndarray, kernel_size: int = 3, sigma: float = 1.0) -> np.ndarray:
    """
    Gaussian blur with thresholding
    Smooths edges but may blur fine details
    """
    print(f"  Applying Gaussian blur (kernel={kernel_size}, sigma={sigma})")

    unknown_mask = map_array == 205

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(map_array.astype(np.float32), (kernel_size, kernel_size), sigma)

    # Re-threshold to get clean values
    result = np.zeros_like(map_array)
    result[blurred > 220] = 254  # free
    result[blurred < 50] = 0     # occupied
    result[(blurred >= 50) & (blurred <= 220)] = 205  # uncertain -> unknown

    # Restore original unknown regions
    result[unknown_mask] = 205

    return result


def apply_remove_isolated(map_array: np.ndarray, min_size: int = 5) -> np.ndarray:
    """
    Remove isolated occupied/free pixels smaller than min_size
    Uses connected component analysis
    """
    print(f"  Removing isolated regions (min_size={min_size})")

    result = map_array.copy()
    unknown_mask = map_array == 205

    # Remove small occupied regions
    occupied_mask = (map_array == 0).astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(occupied_mask, connectivity=8)

    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] < min_size:
            result[labels == i] = 254  # Convert to free space

    # Remove small free regions (holes in walls)
    free_mask = (map_array == 254).astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_mask, connectivity=8)

    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] < min_size:
            result[labels == i] = 0  # Convert to occupied

    result[unknown_mask] = 205
    return result


def apply_wall_thickening(map_array: np.ndarray, thickness: int = 1) -> np.ndarray:
    """
    Thicken walls by dilating occupied regions
    Useful for navigation safety margins
    """
    print(f"  Thickening walls (thickness={thickness})")

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*thickness+1, 2*thickness+1))

    unknown_mask = map_array == 205
    occupied_mask = (map_array == 0).astype(np.uint8)

    # Dilate occupied regions
    dilated = cv2.dilate(occupied_mask, kernel, iterations=1)

    result = map_array.copy()
    result[dilated == 1] = 0  # Make dilated area occupied
    result[unknown_mask] = 205  # Restore unknown

    return result


def apply_edge_cleanup(map_array: np.ndarray) -> np.ndarray:
    """
    Clean up jagged edges on walls using bilateral filter
    Preserves wall structure while smoothing edges
    """
    print("  Cleaning up edges with bilateral filter")

    unknown_mask = map_array == 205

    # Bilateral filter - edge-preserving smoothing
    filtered = cv2.bilateralFilter(map_array, 5, 50, 50)

    # Re-threshold
    result = np.zeros_like(map_array)
    result[filtered > 180] = 254
    result[filtered < 75] = 0
    result[(filtered >= 75) & (filtered <= 180)] = 205

    result[unknown_mask] = 205
    return result


def analyze_map(map_array: np.ndarray):
    """Print statistics about the map"""
    total = map_array.size
    free = np.sum(map_array == 254)
    occupied = np.sum(map_array == 0)
    unknown = np.sum(map_array == 205)
    other = total - free - occupied - unknown

    print("\nMap Statistics:")
    print(f"  Total pixels:    {total:>10}")
    print(f"  Free (254):      {free:>10} ({100*free/total:.1f}%)")
    print(f"  Occupied (0):    {occupied:>10} ({100*occupied/total:.1f}%)")
    print(f"  Unknown (205):   {unknown:>10} ({100*unknown/total:.1f}%)")
    if other > 0:
        print(f"  Other values:    {other:>10} ({100*other/total:.1f}%)")


def main():
    parser = argparse.ArgumentParser(description='Filter noise from SLAM maps')
    parser.add_argument('input', help='Input map YAML file')
    parser.add_argument('--output', '-o', default=None, help='Output base name (default: input_filtered)')
    parser.add_argument('--method', '-m', default='all',
                        choices=['median', 'morpho', 'gaussian', 'isolated', 'walls', 'edges', 'all', 'best'],
                        help='Filtering method to apply')
    parser.add_argument('--kernel', '-k', type=int, default=3, help='Kernel size (default: 3)')
    parser.add_argument('--min-size', type=int, default=5, help='Min region size for isolated removal')
    parser.add_argument('--wall-thickness', type=int, default=1, help='Wall thickening amount')

    args = parser.parse_args()

    # Load map
    map_array, map_info, pgm_path = load_map(args.input)
    analyze_map(map_array)

    # Set output base name
    if args.output is None:
        input_base = Path(args.input).stem
        output_dir = Path(args.input).parent
    else:
        output_dir = Path(args.output).parent or Path('.')
        input_base = Path(args.output).stem

    print(f"\nApplying filter(s): {args.method}")

    if args.method == 'median':
        filtered = apply_median_filter(map_array, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_median"))

    elif args.method == 'morpho':
        filtered = apply_morphological(map_array, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_morpho"))

    elif args.method == 'gaussian':
        filtered = apply_gaussian(map_array, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_gaussian"))

    elif args.method == 'isolated':
        filtered = apply_remove_isolated(map_array, args.min_size)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_isolated"))

    elif args.method == 'walls':
        filtered = apply_wall_thickening(map_array, args.wall_thickness)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_walls"))

    elif args.method == 'edges':
        filtered = apply_edge_cleanup(map_array)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_edges"))

    elif args.method == 'best':
        # Best combination for SLAM noise
        print("\nApplying BEST filter combination:")
        filtered = apply_remove_isolated(map_array, args.min_size)
        filtered = apply_morphological(filtered, args.kernel)
        filtered = apply_median_filter(filtered, args.kernel)
        analyze_map(filtered)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_best"))

    elif args.method == 'all':
        # Save all variations
        print("\nGenerating all filter variations:")

        filtered = apply_median_filter(map_array, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_median"))

        filtered = apply_morphological(map_array, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_morpho"))

        filtered = apply_gaussian(map_array, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_gaussian"))

        filtered = apply_remove_isolated(map_array, args.min_size)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_isolated"))

        filtered = apply_edge_cleanup(map_array)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_edges"))

        # Best combination
        print("\n  Creating BEST combination:")
        filtered = apply_remove_isolated(map_array, args.min_size)
        filtered = apply_morphological(filtered, args.kernel)
        filtered = apply_median_filter(filtered, args.kernel)
        save_map(filtered, map_info, str(output_dir / f"{input_base}_best"))

    print("\nDone! Compare results in RViz or image viewer.")


if __name__ == '__main__':
    main()
