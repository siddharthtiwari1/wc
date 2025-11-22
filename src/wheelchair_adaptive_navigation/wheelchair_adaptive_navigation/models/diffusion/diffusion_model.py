"""
Denoising Diffusion Probabilistic Model (DDPM) for trajectory generation.

Based on "Denoising Diffusion Probabilistic Models" (Ho et al., 2020).
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Tuple, Dict, Optional


class DiffusionModel(nn.Module):
    """
    Complete DDPM for trajectory generation.

    Workflow:
        Training: q(x_t | x_0) - forward diffusion (add noise)
                 p_θ(x_{t-1} | x_t) - reverse diffusion (denoise)

        Sampling: x_T ~ N(0, I)
                 x_0 = reverse_process(x_T)
    """

    def __init__(
        self,
        trajectory_dim: int = 2,      # (x, y) per timestep
        horizon: int = 30,             # Number of trajectory points
        condition_dim: int = 512,      # Perception + goal features
        num_diffusion_steps: int = 1000,
        beta_schedule: str = 'linear',  # 'linear', 'cosine'
        model_channels: int = 128,
        num_layers: int = 4,
    ):
        super().__init__()

        self.trajectory_dim = trajectory_dim
        self.horizon = horizon
        self.condition_dim = condition_dim
        self.num_diffusion_steps = num_diffusion_steps

        # Noise schedule
        self.betas = self._get_beta_schedule(beta_schedule, num_diffusion_steps)
        self.alphas = 1.0 - self.betas
        self.alphas_cumprod = torch.cumprod(self.alphas, dim=0)
        self.alphas_cumprod_prev = F.pad(self.alphas_cumprod[:-1], (1, 0), value=1.0)

        # sqrt(alpha_cumprod) and sqrt(1 - alpha_cumprod)
        self.sqrt_alphas_cumprod = torch.sqrt(self.alphas_cumprod)
        self.sqrt_one_minus_alphas_cumprod = torch.sqrt(1.0 - self.alphas_cumprod)

        # Posterior variance
        self.posterior_variance = (
            self.betas * (1.0 - self.alphas_cumprod_prev) / (1.0 - self.alphas_cumprod)
        )

        # Denoising network (U-Net)
        from .unet_trajectory import UNet1D

        self.denoiser = UNet1D(
            in_channels=trajectory_dim,
            out_channels=trajectory_dim,
            time_emb_dim=128,
            condition_dim=condition_dim,
            model_channels=model_channels,
            num_layers=num_layers,
        )

    def _get_beta_schedule(self, schedule: str, num_steps: int) -> torch.Tensor:
        """Get noise schedule."""
        if schedule == 'linear':
            beta_start = 0.0001
            beta_end = 0.02
            return torch.linspace(beta_start, beta_end, num_steps)

        elif schedule == 'cosine':
            s = 0.008
            steps = num_steps + 1
            x = torch.linspace(0, num_steps, steps)
            alphas_cumprod = torch.cos(((x / num_steps) + s) / (1 + s) * np.pi * 0.5) ** 2
            alphas_cumprod = alphas_cumprod / alphas_cumprod[0]
            betas = 1 - (alphas_cumprod[1:] / alphas_cumprod[:-1])
            return torch.clip(betas, 0.0001, 0.9999)

        else:
            raise ValueError(f"Unknown beta schedule: {schedule}")

    def forward_diffusion(
        self,
        x_0: torch.Tensor,
        t: torch.Tensor,
        noise: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward diffusion: add noise to x_0.

        Args:
            x_0: Clean trajectories [B, T, D]
            t: Timesteps [B]
            noise: Optional noise (for reproducibility)

        Returns:
            x_t: Noised trajectories [B, T, D]
            noise: The noise that was added [B, T, D]
        """
        if noise is None:
            noise = torch.randn_like(x_0)

        # Get alpha values for each sample
        sqrt_alpha_t = self.sqrt_alphas_cumprod[t][:, None, None]
        sqrt_one_minus_alpha_t = self.sqrt_one_minus_alphas_cumprod[t][:, None, None]

        # Add noise: x_t = sqrt(alpha_t) * x_0 + sqrt(1 - alpha_t) * noise
        x_t = sqrt_alpha_t * x_0 + sqrt_one_minus_alpha_t * noise

        return x_t, noise

    def forward(
        self,
        x_0: torch.Tensor,
        condition: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        """
        Training forward pass.

        Args:
            x_0: Clean trajectories [B, T, D]
            condition: Conditioning features [B, condition_dim]

        Returns:
            Dictionary with loss and metrics
        """
        batch_size = x_0.shape[0]
        device = x_0.device

        # Sample random timesteps
        t = torch.randint(0, self.num_diffusion_steps, (batch_size,), device=device)

        # Forward diffusion
        x_t, noise = self.forward_diffusion(x_0, t)

        # Predict noise
        noise_pred = self.denoiser(x_t, t, condition)

        # Loss (MSE between true noise and predicted noise)
        loss = F.mse_loss(noise_pred, noise)

        return {
            'loss': loss,
            'noise_pred': noise_pred,
            'noise_true': noise,
        }

    def reverse_step(
        self,
        x_t: torch.Tensor,
        t: int,
        condition: torch.Tensor,
    ) -> torch.Tensor:
        """
        Single reverse diffusion step.

        Args:
            x_t: Noised trajectory [B, T, D]
            t: Current timestep (scalar)
            condition: Conditioning [B, condition_dim]

        Returns:
            x_{t-1}: Less noised trajectory
        """
        batch_size = x_t.shape[0]
        device = x_t.device

        # Create timestep tensor
        t_tensor = torch.full((batch_size,), t, device=device, dtype=torch.long)

        # Predict noise
        noise_pred = self.denoiser(x_t, t_tensor, condition)

        # Get parameters
        alpha_t = self.alphas[t]
        alpha_cumprod_t = self.alphas_cumprod[t]
        beta_t = self.betas[t]

        # Compute mean
        mean = (1.0 / torch.sqrt(alpha_t)) * (
            x_t - (beta_t / torch.sqrt(1.0 - alpha_cumprod_t)) * noise_pred
        )

        if t > 0:
            # Add noise
            variance = self.posterior_variance[t]
            noise = torch.randn_like(x_t)
            x_t_minus_1 = mean + torch.sqrt(variance) * noise
        else:
            # No noise at final step
            x_t_minus_1 = mean

        return x_t_minus_1

    @torch.no_grad()
    def sample(
        self,
        batch_size: int,
        condition: torch.Tensor,
        device: str = 'cpu',
    ) -> torch.Tensor:
        """
        Generate trajectories via reverse diffusion.

        Args:
            batch_size: Number of samples
            condition: Conditioning [batch_size, condition_dim]
            device: torch device

        Returns:
            trajectories: [batch_size, horizon, trajectory_dim]
        """
        # Start from pure noise
        x_t = torch.randn(
            batch_size, self.horizon, self.trajectory_dim,
            device=device,
        )

        # Reverse diffusion
        for t in reversed(range(self.num_diffusion_steps)):
            x_t = self.reverse_step(x_t, t, condition)

        return x_t

    @torch.no_grad()
    def sample_ddim(
        self,
        batch_size: int,
        condition: torch.Tensor,
        num_steps: int = 50,
        device: str = 'cpu',
    ) -> torch.Tensor:
        """
        Fast sampling using DDIM (deterministic).

        Args:
            batch_size: Number of samples
            condition: Conditioning
            num_steps: Number of denoising steps (< num_diffusion_steps)
            device: torch device

        Returns:
            trajectories: [batch_size, horizon, trajectory_dim]
        """
        # Timestep schedule
        skip = self.num_diffusion_steps // num_steps
        timesteps = torch.arange(0, self.num_diffusion_steps, skip, device=device)

        # Start from noise
        x_t = torch.randn(batch_size, self.horizon, self.trajectory_dim, device=device)

        # DDIM reverse process
        for i in reversed(range(len(timesteps))):
            t = timesteps[i]

            # Predict noise
            t_tensor = torch.full((batch_size,), t, device=device, dtype=torch.long)
            noise_pred = self.denoiser(x_t, t_tensor, condition)

            # DDIM update (deterministic)
            alpha_t = self.alphas_cumprod[t]

            if i > 0:
                alpha_t_prev = self.alphas_cumprod[timesteps[i - 1]]
            else:
                alpha_t_prev = torch.tensor(1.0, device=device)

            # Predict x_0
            x_0_pred = (x_t - torch.sqrt(1.0 - alpha_t) * noise_pred) / torch.sqrt(alpha_t)

            # Compute x_{t-1}
            x_t = torch.sqrt(alpha_t_prev) * x_0_pred + torch.sqrt(1.0 - alpha_t_prev) * noise_pred

        return x_t


def test_diffusion_model():
    """Unit test for diffusion model."""
    batch_size = 4
    horizon = 30
    trajectory_dim = 2
    condition_dim = 512

    model = DiffusionModel(
        trajectory_dim=trajectory_dim,
        horizon=horizon,
        condition_dim=condition_dim,
        num_diffusion_steps=100,  # Small for testing
    )

    # Test data
    x_0 = torch.randn(batch_size, horizon, trajectory_dim)
    condition = torch.randn(batch_size, condition_dim)

    # Forward pass (training)
    output = model(x_0, condition)

    print(f"Training loss: {output['loss']:.4f}")
    assert 'loss' in output

    # Sampling
    samples = model.sample(batch_size=2, condition=condition[:2], device='cpu')

    print(f"Samples shape: {samples.shape}")
    assert samples.shape == (2, horizon, trajectory_dim)

    # DDIM sampling (faster)
    samples_ddim = model.sample_ddim(
        batch_size=2,
        condition=condition[:2],
        num_steps=20,
        device='cpu',
    )

    print(f"DDIM samples shape: {samples_ddim.shape}")
    assert samples_ddim.shape == (2, horizon, trajectory_dim)

    print("✓ Diffusion model test passed!")


if __name__ == '__main__':
    test_diffusion_model()
