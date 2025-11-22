"""Noise scheduler utilities."""

import torch


class NoiseScheduler:
    """Helper class for managing noise schedules."""

    def __init__(self, num_steps=1000, beta_schedule='linear'):
        self.num_steps = num_steps
        self.beta_schedule = beta_schedule

    def get_schedule(self):
        """Get beta schedule."""
        # Placeholder - full implementation in diffusion_model.py
        pass


__all__ = ['NoiseScheduler']
