"""Crowd flow prediction module."""

import torch.nn as nn


class FlowPredictor(nn.Module):
    """Predict crowd flow patterns."""

    def __init__(self, input_dim=512, output_dim=2):
        super().__init__()
        self.predictor = nn.Sequential(
            nn.Linear(input_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim),
        )

    def forward(self, features):
        """Predict flow vector."""
        return self.predictor(features)


__all__ = ['FlowPredictor']
