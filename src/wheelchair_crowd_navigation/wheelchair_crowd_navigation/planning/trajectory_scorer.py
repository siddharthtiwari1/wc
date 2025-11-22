"""
Trajectory scoring and cost functions.
"""

import torch
import numpy as np
from typing import List, Dict, Optional


class CostFunction:
    """
    Multi-objective cost function for trajectory evaluation.

    Combines:
        - Goal-reaching cost
        - Collision cost
        - Smoothness cost
        - Comfort cost (jerk, acceleration)
    """

    def __init__(
        self,
        goal_weight: float = 1.0,
        collision_weight: float = 10.0,
        smoothness_weight: float = 0.1,
        comfort_weight: float = 0.05,
        goal_tolerance: float = 0.5,
        collision_margin: float = 0.5,
        dt: float = 0.1,
    ):
        self.goal_weight = goal_weight
        self.collision_weight = collision_weight
        self.smoothness_weight = smoothness_weight
        self.comfort_weight = comfort_weight

        self.goal_tolerance = goal_tolerance
        self.collision_margin = collision_margin
        self.dt = dt

    def __call__(
        self,
        trajectories: torch.Tensor,
        goal: torch.Tensor,
        obstacles: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """
        Compute total cost for trajectories.

        Args:
            trajectories: [N, T, 2] batch of trajectories
            goal: [2] or [3] goal position (x, y) or (x, y, theta)
            obstacles: [M, 2] obstacle positions

        Returns:
            costs: [N] cost for each trajectory
        """
        costs = torch.zeros(trajectories.shape[0], device=trajectories.device)

        # Goal-reaching cost
        costs += self.goal_weight * self._goal_cost(trajectories, goal)

        # Collision cost
        if obstacles is not None:
            costs += self.collision_weight * self._collision_cost(trajectories, obstacles)

        # Smoothness cost
        costs += self.smoothness_weight * self._smoothness_cost(trajectories)

        # Comfort cost
        costs += self.comfort_weight * self._comfort_cost(trajectories)

        return costs

    def _goal_cost(self, trajectories: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """Distance to goal at end of trajectory."""
        final_pos = trajectories[:, -1, :2]  # [N, 2]
        goal_pos = goal[:2]

        distances = torch.norm(final_pos - goal_pos, dim=-1)
        return distances

    def _collision_cost(self, trajectories: torch.Tensor, obstacles: torch.Tensor) -> torch.Tensor:
        """Penalty for getting too close to obstacles."""
        # trajectories: [N, T, 2]
        # obstacles: [M, 2]

        N, T, _ = trajectories.shape
        M = obstacles.shape[0]

        # Expand for broadcasting
        traj_expanded = trajectories.unsqueeze(2)  # [N, T, 1, 2]
        obs_expanded = obstacles.unsqueeze(0).unsqueeze(0)  # [1, 1, M, 2]

        # Distances to all obstacles at all timesteps
        distances = torch.norm(traj_expanded - obs_expanded, dim=-1)  # [N, T, M]

        # Minimum distance to any obstacle at any time
        min_distances = distances.min(dim=2)[0]  # [N, T]

        # Penalty for being too close
        violations = torch.clamp(self.collision_margin - min_distances, min=0.0)

        # Sum over time
        collision_cost = violations.sum(dim=1)  # [N]

        return collision_cost

    def _smoothness_cost(self, trajectories: torch.Tensor) -> torch.Tensor:
        """Penalty for non-smooth trajectories (large velocity changes)."""
        # Compute velocities
        velocities = torch.diff(trajectories, dim=1) / self.dt  # [N, T-1, 2]

        # Compute accelerations
        accelerations = torch.diff(velocities, dim=1) / self.dt  # [N, T-2, 2]

        # L2 norm of accelerations
        smoothness_cost = torch.norm(accelerations, dim=-1).sum(dim=1)  # [N]

        return smoothness_cost

    def _comfort_cost(self, trajectories: torch.Tensor) -> torch.Tensor:
        """Penalty for jerky motion (large acceleration changes)."""
        # Compute velocities and accelerations
        velocities = torch.diff(trajectories, dim=1) / self.dt  # [N, T-1, 2]
        accelerations = torch.diff(velocities, dim=1) / self.dt  # [N, T-2, 2]

        # Compute jerk
        jerk = torch.diff(accelerations, dim=1) / self.dt  # [N, T-3, 2]

        # L2 norm of jerk
        comfort_cost = torch.norm(jerk, dim=-1).sum(dim=1)  # [N]

        return comfort_cost


class TrajectoryScorer:
    """
    High-level trajectory scorer with perception integration.
    """

    def __init__(self, config: Optional[Dict] = None):
        config = config or {}

        self.cost_function = CostFunction(
            goal_weight=config.get('goal_weight', 1.0),
            collision_weight=config.get('collision_weight', 10.0),
            smoothness_weight=config.get('smoothness_weight', 0.1),
            comfort_weight=config.get('comfort_weight', 0.05),
        )

    def score_trajectories(
        self,
        trajectories: torch.Tensor,
        goal: torch.Tensor,
        laser_scan: Optional[np.ndarray] = None,
        point_cloud: Optional[np.ndarray] = None,
    ) -> torch.Tensor:
        """
        Score trajectories given perception data.

        Args:
            trajectories: [N, T, 2]
            goal: [2] or [3]
            laser_scan: LaserScan data
            point_cloud: PointCloud data

        Returns:
            costs: [N]
        """
        # Extract obstacles from perception
        obstacles = self._extract_obstacles(laser_scan, point_cloud)

        # Compute costs
        costs = self.cost_function(trajectories, goal, obstacles)

        return costs

    def _extract_obstacles(
        self,
        laser_scan: Optional[np.ndarray],
        point_cloud: Optional[np.ndarray],
    ) -> Optional[torch.Tensor]:
        """Extract obstacle positions from perception data."""
        if laser_scan is not None:
            return self._laser_to_obstacles(laser_scan)
        elif point_cloud is not None:
            return self._pointcloud_to_obstacles(point_cloud)
        else:
            return None

    def _laser_to_obstacles(self, laser_scan: np.ndarray) -> torch.Tensor:
        """Convert laser scan to obstacle points."""
        # laser_scan: [num_rays] with ranges

        num_rays = len(laser_scan)
        angles = np.linspace(-np.pi, np.pi, num_rays)

        # Convert to Cartesian
        x = laser_scan * np.cos(angles)
        y = laser_scan * np.sin(angles)

        # Filter invalid points
        valid = (laser_scan > 0.1) & (laser_scan < 10.0)
        x = x[valid]
        y = y[valid]

        obstacles = np.stack([x, y], axis=1)  # [M, 2]

        return torch.from_numpy(obstacles).float()

    def _pointcloud_to_obstacles(self, point_cloud: np.ndarray) -> torch.Tensor:
        """Convert point cloud to obstacle points."""
        # point_cloud: [num_points, 3] (x, y, z)

        # Take only x, y
        obstacles = point_cloud[:, :2]

        return torch.from_numpy(obstacles).float()


if __name__ == '__main__':
    # Test cost function
    cost_fn = CostFunction()

    trajectories = torch.randn(10, 30, 2)  # 10 trajectories
    goal = torch.tensor([5.0, 0.0])
    obstacles = torch.randn(20, 2)  # 20 obstacles

    costs = cost_fn(trajectories, goal, obstacles)

    print(f"Costs shape: {costs.shape}")
    print(f"Costs: {costs}")

    assert costs.shape == (10,)

    print("✓ Trajectory scorer test passed!")
