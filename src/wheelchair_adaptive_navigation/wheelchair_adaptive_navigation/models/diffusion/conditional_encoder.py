"""Conditional encoder for context."""

import torch.nn as nn


class ConditionalEncoder(nn.Module):
    """Encode perception + goal into conditioning vector."""

    def __init__(self, perception_dim=512, goal_dim=3, output_dim=512):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(perception_dim + goal_dim, 256),
            nn.ReLU(),
            nn.Linear(256, output_dim),
        )

    def forward(self, perception, goal):
        """
        Args:
            perception: [B, perception_dim]
            goal: [B, goal_dim]
        Returns:
            condition: [B, output_dim]
        """
        import torch
        x = torch.cat([perception, goal], dim=-1)
        return self.encoder(x)


__all__ = ['ConditionalEncoder']
