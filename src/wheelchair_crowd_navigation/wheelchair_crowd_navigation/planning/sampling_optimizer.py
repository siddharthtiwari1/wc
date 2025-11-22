"""
Sampling-based optimizer for trajectory refinement.

Uses Cross-Entropy Method (CEM) or other sampling strategies.
"""

import torch
import numpy as np
from typing import Callable, Tuple, Optional


class CrossEntropyMethod:
    """
    Cross-Entropy Method for trajectory optimization.

    Iteratively samples from a distribution and refines it
    using the best samples (elites).
    """

    def __init__(
        self,
        num_samples: int = 100,
        num_elites: int = 10,
        num_iterations: int = 5,
        temperature: float = 1.0,
        alpha: float = 0.3,  # Smoothing factor
    ):
        self.num_samples = num_samples
        self.num_elites = num_elites
        self.num_iterations = num_iterations
        self.temperature = temperature
        self.alpha = alpha

    def optimize(
        self,
        initial_samples: torch.Tensor,
        cost_function: Callable[[torch.Tensor], torch.Tensor],
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Optimize using CEM.

        Args:
            initial_samples: [num_samples, ...] initial trajectory samples
            cost_function: Function that scores trajectories (lower is better)

        Returns:
            best_sample: Best trajectory
            best_cost: Cost of best trajectory
        """
        samples = initial_samples
        device = samples.device

        # Compute statistics
        mean = samples.mean(dim=0, keepdim=True)
        std = samples.std(dim=0, keepdim=True) + 1e-6

        for iteration in range(self.num_iterations):
            # Evaluate costs
            costs = cost_function(samples)  # [num_samples]

            # Select elites (lowest cost)
            elite_indices = torch.argsort(costs)[:self.num_elites]
            elites = samples[elite_indices]

            # Update distribution (with smoothing)
            new_mean = elites.mean(dim=0, keepdim=True)
            new_std = elites.std(dim=0, keepdim=True) + 1e-6

            mean = self.alpha * new_mean + (1 - self.alpha) * mean
            std = self.alpha * new_std + (1 - self.alpha) * std

            # Sample new candidates (except last iteration)
            if iteration < self.num_iterations - 1:
                noise = torch.randn_like(samples)
                samples = mean + self.temperature * std * noise

        # Return best elite
        final_costs = cost_function(elites)
        best_idx = torch.argmin(final_costs)

        return elites[best_idx], final_costs[best_idx]


class SamplingOptimizer:
    """
    Main sampling optimizer that combines VQVAE prior with CEM.

    Workflow:
        1. Sample codes from PixelCNN prior
        2. Decode to trajectories
        3. Refine with CEM
    """

    def __init__(
        self,
        vqvae_model,
        pixelcnn_model: Optional = None,
        num_samples: int = 100,
        num_elites: int = 10,
        num_iterations: int = 5,
        use_prior: bool = True,
    ):
        self.vqvae = vqvae_model
        self.pixelcnn = pixelcnn_model
        self.use_prior = use_prior and pixelcnn_model is not None

        self.cem = CrossEntropyMethod(
            num_samples=num_samples,
            num_elites=num_elites,
            num_iterations=num_iterations,
        )

    def generate_initial_samples(
        self,
        goal: torch.Tensor,
        num_samples: int,
        device: str = 'cpu',
    ) -> torch.Tensor:
        """
        Generate initial trajectory samples.

        Args:
            goal: Goal state [batch_size, goal_dim]
            num_samples: Number of samples
            device: torch device

        Returns:
            trajectories: [num_samples, horizon, output_dim]
        """
        batch_size = goal.shape[0]
        assert batch_size == 1, "Only single goal supported for now"

        if self.use_prior and self.pixelcnn is not None:
            # Sample codes from PixelCNN prior
            codes = self.pixelcnn.sample(
                batch_size=num_samples,
                length=1,
                condition=goal.repeat(num_samples, 1),
                temperature=1.0,
                device=device,
            )  # [num_samples, 1]

            # Decode to trajectories
            with torch.no_grad():
                trajectories = self.vqvae.decode_codes(
                    codes.squeeze(1),
                    goal.repeat(num_samples, 1),
                )

        else:
            # Sample random codes from codebook
            num_embeddings = self.vqvae.quantizer.num_embeddings
            codes = torch.randint(
                0, num_embeddings,
                (num_samples,),
                device=device,
            )

            # Decode
            with torch.no_grad():
                trajectories = self.vqvae.decode_codes(
                    codes,
                    goal.repeat(num_samples, 1),
                )

        return trajectories

    def optimize(
        self,
        goal: torch.Tensor,
        cost_function: Callable,
        device: str = 'cpu',
    ) -> Tuple[torch.Tensor, float]:
        """
        Full optimization pipeline.

        Args:
            goal: Goal state [1, goal_dim]
            cost_function: Function to evaluate trajectories
            device: torch device

        Returns:
            best_trajectory: [horizon, output_dim]
            cost: float
        """
        # Generate initial samples from VQVAE/PixelCNN
        initial_samples = self.generate_initial_samples(
            goal,
            num_samples=self.cem.num_samples,
            device=device,
        )

        # Optimize with CEM
        best_trajectory, best_cost = self.cem.optimize(
            initial_samples,
            cost_function,
        )

        return best_trajectory, best_cost.item()


def test_sampling_optimizer():
    """Unit test for sampling optimizer."""
    # Mock VQVAE
    from wheelchair_crowd_navigation.models.vqvae import VQVAE

    vqvae = VQVAE(
        perception_dim=512,
        goal_dim=3,
        latent_dim=64,
        num_embeddings=128,
        horizon=30,
    )

    # Dummy cost function
    def cost_fn(trajectories):
        # Simple cost: distance to goal
        goal_pos = torch.tensor([5.0, 0.0])
        final_pos = trajectories[:, -1, :]  # [N, 2]
        costs = torch.norm(final_pos - goal_pos, dim=-1)
        return costs

    optimizer = SamplingOptimizer(
        vqvae_model=vqvae,
        pixelcnn_model=None,
        num_samples=50,
        num_elites=5,
        num_iterations=3,
    )

    goal = torch.tensor([[5.0, 0.0, 0.0]])  # (x, y, theta)

    best_traj, cost = optimizer.optimize(
        goal=goal,
        cost_function=cost_fn,
        device='cpu',
    )

    print(f"Best trajectory shape: {best_traj.shape}")
    print(f"Best cost: {cost:.4f}")

    assert best_traj.shape == (30, 2)
    assert cost < 10.0  # Should reach reasonably close to goal

    print("✓ Sampling optimizer test passed!")


if __name__ == '__main__':
    test_sampling_optimizer()
