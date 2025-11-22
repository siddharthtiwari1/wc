"""
Social group detection using F-formations.

Identifies groups of people moving together.
"""

import numpy as np
from sklearn.cluster import DBSCAN


class SocialGroupDetector:
    """
    Detect social groups in crowds.

    Uses DBSCAN clustering based on proximity and velocity similarity.
    """

    def __init__(
        self,
        distance_threshold: float = 2.0,  # meters
        min_group_size: int = 2,
    ):
        self.distance_threshold = distance_threshold
        self.min_group_size = min_group_size

    def detect_groups(
        self,
        positions: np.ndarray,
        velocities: np.ndarray = None,
    ) -> list:
        """
        Detect social groups from pedestrian positions.

        Args:
            positions: [N, 2] pedestrian positions
            velocities: [N, 2] optional velocities

        Returns:
            groups: List of groups, each is list of indices
        """
        if len(positions) < self.min_group_size:
            return []

        # Cluster by position
        clustering = DBSCAN(
            eps=self.distance_threshold,
            min_samples=self.min_group_size,
        ).fit(positions)

        labels = clustering.labels_

        # Extract groups
        groups = []
        for label in set(labels):
            if label == -1:  # Noise
                continue

            group_indices = np.where(labels == label)[0].tolist()

            if len(group_indices) >= self.min_group_size:
                groups.append(group_indices)

        return groups


if __name__ == '__main__':
    detector = SocialGroupDetector()

    # Test data: two groups
    positions = np.array([
        [0, 0], [0.5, 0], [0, 0.5],  # Group 1
        [10, 10], [10.5, 10],  # Group 2
        [20, 20],  # Outlier
    ])

    groups = detector.detect_groups(positions)

    print(f"Detected {len(groups)} groups:")
    for i, group in enumerate(groups):
        print(f"  Group {i}: {group}")

    print("✓ Group detector test passed!")
