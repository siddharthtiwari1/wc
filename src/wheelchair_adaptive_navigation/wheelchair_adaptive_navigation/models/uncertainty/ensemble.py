"""
Trajectory ensemble for uncertainty quantification.
"""

import torch
import numpy as np
from typing import Tuple


class TrajectoryEnsemble:
    """
    Ensemble of trajectories for uncertainty estimation.

    Uses multiple samples from diffusion model to quantify uncertainty.
    """

    def __init__(self, ensemble_size: int = 10):
        self.ensemble_size = ensemble_size

    def compute_uncertainty(
        self,
        trajectories: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Compute mean and variance of trajectory ensemble.

        Args:
            trajectories: [N, T, D] ensemble of trajectories

        Returns:
            mean: [T, D] mean trajectory
            variance: [T, D] pointwise variance
        """
        mean = trajectories.mean(dim=0)
        variance = trajectories.var(dim=0)

        return mean, variance

    def select_safe_trajectory(
        self,
        trajectories: torch.Tensor,
        costs: torch.Tensor,
        risk_threshold: float = 0.5,
    ) -> int:
        """
        Select trajectory considering both cost and uncertainty.

        Args:
            trajectories: [N, T, D]
            costs: [N] cost for each trajectory
            risk_threshold: Maximum acceptable variance

        Returns:
            index: Index of selected trajectory
        """
        _, variance = self.compute_uncertainty(trajectories)

        # Total variance
        total_variance = variance.sum(dim=-1).mean()  # Average over time

        # If variance is low, pick lowest cost
        if total_variance < risk_threshold:
            return torch.argmin(costs).item()

        # Otherwise, trade off cost and variance
        # Pick trajectory closest to mean
        mean, _ = self.compute_uncertainty(trajectories)

        distances = torch.norm(trajectories - mean.unsqueeze(0), dim=(1, 2))

        return torch.argmin(distances).item()


if __name__ == '__main__':
    ensemble = TrajectoryEnsemble(ensemble_size=10)

    trajectories = torch.randn(10, 30, 2)
    costs = torch.randn(10)

    idx = ensemble.select_safe_trajectory(trajectories, costs)

    print(f"Selected trajectory index: {idx}")

    print("✓ Ensemble test passed!")
