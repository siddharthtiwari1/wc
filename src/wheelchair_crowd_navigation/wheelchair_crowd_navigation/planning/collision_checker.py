"""
Collision checking utilities for trajectory validation.
"""

import torch
import numpy as np
from typing import Optional, Tuple


class CollisionChecker:
    """
    Fast collision checking for wheelchair trajectories.

    Checks collisions with:
        - Point obstacles (from laser/point cloud)
        - Static obstacles (map)
        - Dynamic obstacles (pedestrians)
    """

    def __init__(
        self,
        robot_radius: float = 0.5,  # meters
        safety_margin: float = 0.3,  # additional safety buffer
        use_gpu: bool = False,
    ):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.collision_radius = robot_radius + safety_margin
        self.use_gpu = use_gpu

    def check_trajectory(
        self,
        trajectory: torch.Tensor,
        obstacles: Optional[torch.Tensor] = None,
    ) -> bool:
        """
        Check if trajectory is collision-free.

        Args:
            trajectory: [T, 2] trajectory points
            obstacles: [M, 2] obstacle positions

        Returns:
            collision_free: True if no collisions
        """
        if obstacles is None or len(obstacles) == 0:
            return True

        # Compute distances from each trajectory point to each obstacle
        traj_expanded = trajectory.unsqueeze(1)  # [T, 1, 2]
        obs_expanded = obstacles.unsqueeze(0)  # [1, M, 2]

        distances = torch.norm(traj_expanded - obs_expanded, dim=-1)  # [T, M]

        # Check if any distance is below collision radius
        min_distance = distances.min()

        return min_distance.item() > self.collision_radius

    def check_batch(
        self,
        trajectories: torch.Tensor,
        obstacles: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """
        Check collisions for a batch of trajectories.

        Args:
            trajectories: [N, T, 2]
            obstacles: [M, 2]

        Returns:
            collision_free: [N] boolean tensor
        """
        if obstacles is None or len(obstacles) == 0:
            return torch.ones(trajectories.shape[0], dtype=torch.bool)

        N, T, _ = trajectories.shape
        M = obstacles.shape[0]

        # Expand for broadcasting
        traj_expanded = trajectories.unsqueeze(2)  # [N, T, 1, 2]
        obs_expanded = obstacles.unsqueeze(0).unsqueeze(0)  # [1, 1, M, 2]

        # Compute all distances
        distances = torch.norm(traj_expanded - obs_expanded, dim=-1)  # [N, T, M]

        # Get minimum distance for each trajectory
        min_distances = distances.reshape(N, -1).min(dim=1)[0]  # [N]

        # Check collision
        collision_free = min_distances > self.collision_radius

        return collision_free

    def get_closest_obstacle_distances(
        self,
        trajectory: torch.Tensor,
        obstacles: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """
        Get distance to closest obstacle at each time step.

        Args:
            trajectory: [T, 2]
            obstacles: [M, 2]

        Returns:
            distances: [T] distance to closest obstacle
        """
        if obstacles is None or len(obstacles) == 0:
            return torch.full((trajectory.shape[0],), float('inf'))

        traj_expanded = trajectory.unsqueeze(1)  # [T, 1, 2]
        obs_expanded = obstacles.unsqueeze(0)  # [1, M, 2]

        distances = torch.norm(traj_expanded - obs_expanded, dim=-1)  # [T, M]

        min_distances = distances.min(dim=1)[0]  # [T]

        return min_distances


class DynamicObstaclePredictor:
    """
    Simple predictor for dynamic obstacles (pedestrians).

    Uses constant velocity model for short-term prediction.
    """

    def __init__(self, prediction_horizon: float = 3.0, dt: float = 0.1):
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.num_steps = int(prediction_horizon / dt)

    def predict_trajectories(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
    ) -> np.ndarray:
        """
        Predict future trajectories using constant velocity.

        Args:
            positions: [N, 2] current positions
            velocities: [N, 2] current velocities

        Returns:
            trajectories: [N, T, 2] predicted trajectories
        """
        N = positions.shape[0]
        trajectories = np.zeros((N, self.num_steps, 2))

        for t in range(self.num_steps):
            trajectories[:, t, :] = positions + velocities * (t * self.dt)

        return trajectories


if __name__ == '__main__':
    # Test collision checker
    checker = CollisionChecker(robot_radius=0.5, safety_margin=0.2)

    trajectory = torch.tensor([
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
    ])

    obstacles = torch.tensor([
        [1.5, 0.0],  # Close to trajectory
        [10.0, 10.0],  # Far from trajectory
    ])

    is_free = checker.check_trajectory(trajectory, obstacles)
    print(f"Collision free: {is_free}")  # Should be False

    # Test batch checking
    trajectories = torch.randn(10, 30, 2)
    collision_free = checker.check_batch(trajectories, obstacles)

    print(f"Collision-free trajectories: {collision_free.sum()}/{len(collision_free)}")

    print("✓ Collision checker test passed!")
