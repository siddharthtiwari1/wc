"""Risk estimation from trajectory uncertainty."""

import torch


class RiskEstimator:
    """Estimate collision risk from trajectory uncertainty."""

    def __init__(self, risk_threshold: float = 0.5):
        self.risk_threshold = risk_threshold

    def estimate_risk(
        self,
        trajectory_mean: torch.Tensor,
        trajectory_variance: torch.Tensor,
        obstacles: torch.Tensor,
    ) -> torch.Tensor:
        """
        Estimate collision risk.

        Args:
            trajectory_mean: [T, 2]
            trajectory_variance: [T, 2]
            obstacles: [M, 2]

        Returns:
            risk: Scalar risk value
        """
        # Simple risk: probability of collision
        # (Gaussian approximation)

        # Compute distances to obstacles
        mean_expanded = trajectory_mean.unsqueeze(1)  # [T, 1, 2]
        obs_expanded = obstacles.unsqueeze(0)  # [1, M, 2]

        distances = torch.norm(mean_expanded - obs_expanded, dim=-1)  # [T, M]

        # Minimum distance at each time
        min_distances = distances.min(dim=1)[0]  # [T]

        # Variance along trajectory
        position_std = torch.sqrt(trajectory_variance.sum(dim=-1))  # [T]

        # Risk: probability that true position violates safety margin
        # P(d < margin) assuming Gaussian
        safety_margin = 0.5

        risk = torch.sigmoid((safety_margin - min_distances) / position_std)

        return risk.mean()


__all__ = ['RiskEstimator']
