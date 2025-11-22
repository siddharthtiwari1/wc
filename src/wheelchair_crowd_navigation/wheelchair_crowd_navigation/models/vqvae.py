"""
Vector Quantized Variational Autoencoder (VQVAE) for trajectory generation.

Based on "Neural Discrete Representation Learning" (van den Oord et al., 2017)
and adapted for the CrowdSurfer approach.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple, Dict, Optional


class VectorQuantizer(nn.Module):
    """
    Vector Quantization layer with exponential moving average updates.

    Args:
        num_embeddings: Size of the codebook (number of discrete codes)
        embedding_dim: Dimension of each code vector
        commitment_cost: Weight for commitment loss
        decay: Decay rate for EMA updates (0.0 = no EMA)
    """

    def __init__(
        self,
        num_embeddings: int = 512,
        embedding_dim: int = 64,
        commitment_cost: float = 0.25,
        decay: float = 0.99,
    ):
        super().__init__()

        self.num_embeddings = num_embeddings
        self.embedding_dim = embedding_dim
        self.commitment_cost = commitment_cost
        self.decay = decay

        # Codebook embeddings
        self.embedding = nn.Embedding(num_embeddings, embedding_dim)
        self.embedding.weight.data.uniform_(-1.0 / num_embeddings, 1.0 / num_embeddings)

        # EMA parameters
        if decay > 0.0:
            self.register_buffer('ema_cluster_size', torch.zeros(num_embeddings))
            self.register_buffer('ema_w', torch.clone(self.embedding.weight.data))

    def forward(
        self,
        z: torch.Tensor,
    ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        """
        Quantize continuous latents to discrete codes.

        Args:
            z: Continuous latents [B, D] or [B, T, D]

        Returns:
            z_q: Quantized latents (same shape as z)
            info: Dictionary with 'loss', 'perplexity', 'encodings'
        """
        # Flatten if needed
        input_shape = z.shape
        flat_z = z.view(-1, self.embedding_dim)  # [B*T, D]

        # Calculate distances to codebook vectors
        # ||z - e||^2 = ||z||^2 + ||e||^2 - 2 * z^T e
        distances = (
            torch.sum(flat_z ** 2, dim=1, keepdim=True)
            + torch.sum(self.embedding.weight ** 2, dim=1)
            - 2 * torch.matmul(flat_z, self.embedding.weight.t())
        )  # [B*T, K]

        # Get nearest codebook entries
        encoding_indices = torch.argmin(distances, dim=1)  # [B*T]
        encodings = F.one_hot(encoding_indices, self.num_embeddings).float()  # [B*T, K]

        # Quantize and unflatten
        quantized = torch.matmul(encodings, self.embedding.weight)  # [B*T, D]
        quantized = quantized.view(input_shape)  # Restore original shape

        # Loss
        e_latent_loss = F.mse_loss(quantized.detach(), z)
        q_latent_loss = F.mse_loss(quantized, z.detach())
        loss = q_latent_loss + self.commitment_cost * e_latent_loss

        # Straight-through estimator
        quantized = z + (quantized - z).detach()

        # Perplexity (measure of codebook usage)
        avg_probs = torch.mean(encodings, dim=0)
        perplexity = torch.exp(-torch.sum(avg_probs * torch.log(avg_probs + 1e-10)))

        # EMA codebook update
        if self.training and self.decay > 0.0:
            self._ema_update(encodings, flat_z)

        info = {
            'loss': loss,
            'perplexity': perplexity,
            'encodings': encoding_indices.view(input_shape[:-1]),  # [B] or [B, T]
            'min_distance': torch.min(distances, dim=1)[0].mean(),
        }

        return quantized, info

    def _ema_update(self, encodings: torch.Tensor, flat_z: torch.Tensor):
        """Update codebook using exponential moving average."""
        with torch.no_grad():
            # Update cluster sizes
            cluster_size = torch.sum(encodings, dim=0)
            self.ema_cluster_size.mul_(self.decay).add_(
                cluster_size, alpha=1 - self.decay
            )

            # Laplace smoothing
            n = torch.sum(self.ema_cluster_size)
            cluster_size = (
                (self.ema_cluster_size + 1e-5) /
                (n + self.num_embeddings * 1e-5) * n
            )

            # Update embeddings
            dw = torch.matmul(encodings.t(), flat_z)
            self.ema_w.mul_(self.decay).add_(dw, alpha=1 - self.decay)
            self.embedding.weight.data.copy_(self.ema_w / cluster_size.unsqueeze(1))


class VQVAEEncoder(nn.Module):
    """
    Encoder network for VQVAE.

    Takes perception features and goal, outputs continuous latents.
    """

    def __init__(
        self,
        input_dim: int = 512,  # Perception features
        goal_dim: int = 3,     # (x, y, theta)
        hidden_dims: list = [256, 128],
        latent_dim: int = 64,
    ):
        super().__init__()

        layers = []
        prev_dim = input_dim + goal_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(inplace=True),
                nn.LayerNorm(hidden_dim),
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, latent_dim))

        self.encoder = nn.Sequential(*layers)

    def forward(self, perception_features: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """
        Args:
            perception_features: [B, input_dim]
            goal: [B, goal_dim]

        Returns:
            z: [B, latent_dim]
        """
        x = torch.cat([perception_features, goal], dim=-1)
        return self.encoder(x)


class VQVAEDecoder(nn.Module):
    """
    Decoder network for VQVAE.

    Takes quantized latents and goal, outputs trajectory.
    """

    def __init__(
        self,
        latent_dim: int = 64,
        goal_dim: int = 3,
        hidden_dims: list = [128, 256],
        horizon: int = 30,      # Number of trajectory points
        output_dim: int = 2,    # (x, y) per point
    ):
        super().__init__()

        self.horizon = horizon
        self.output_dim = output_dim

        layers = []
        prev_dim = latent_dim + goal_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(inplace=True),
                nn.LayerNorm(hidden_dim),
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, horizon * output_dim))

        self.decoder = nn.Sequential(*layers)

    def forward(self, z_q: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """
        Args:
            z_q: Quantized latents [B, latent_dim]
            goal: [B, goal_dim]

        Returns:
            trajectory: [B, horizon, output_dim]
        """
        x = torch.cat([z_q, goal], dim=-1)
        output = self.decoder(x)
        trajectory = output.view(-1, self.horizon, self.output_dim)
        return trajectory


class VQVAE(nn.Module):
    """
    Complete VQVAE model for trajectory generation.

    Architecture:
        Encoder: (perception, goal) -> z_e
        Quantizer: z_e -> z_q (discrete)
        Decoder: (z_q, goal) -> trajectory
    """

    def __init__(
        self,
        perception_dim: int = 512,
        goal_dim: int = 3,
        encoder_hidden: list = [256, 128],
        decoder_hidden: list = [128, 256],
        latent_dim: int = 64,
        num_embeddings: int = 512,
        commitment_cost: float = 0.25,
        horizon: int = 30,
        output_dim: int = 2,
    ):
        super().__init__()

        self.encoder = VQVAEEncoder(
            input_dim=perception_dim,
            goal_dim=goal_dim,
            hidden_dims=encoder_hidden,
            latent_dim=latent_dim,
        )

        self.quantizer = VectorQuantizer(
            num_embeddings=num_embeddings,
            embedding_dim=latent_dim,
            commitment_cost=commitment_cost,
        )

        self.decoder = VQVAEDecoder(
            latent_dim=latent_dim,
            goal_dim=goal_dim,
            hidden_dims=decoder_hidden,
            horizon=horizon,
            output_dim=output_dim,
        )

        self.horizon = horizon
        self.output_dim = output_dim

    def forward(
        self,
        perception: torch.Tensor,
        goal: torch.Tensor,
    ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        """
        Forward pass through VQVAE.

        Args:
            perception: Perception features [B, perception_dim]
            goal: Goal state [B, goal_dim]

        Returns:
            trajectory: Reconstructed trajectory [B, horizon, output_dim]
            info: Dictionary with losses and metrics
        """
        # Encode
        z_e = self.encoder(perception, goal)

        # Quantize
        z_q, quant_info = self.quantizer(z_e)

        # Decode
        trajectory = self.decoder(z_q, goal)

        # Combine info
        info = {
            'vq_loss': quant_info['loss'],
            'perplexity': quant_info['perplexity'],
            'encodings': quant_info['encodings'],
            'min_distance': quant_info['min_distance'],
        }

        return trajectory, info

    def encode(
        self,
        perception: torch.Tensor,
        goal: torch.Tensor,
    ) -> torch.Tensor:
        """Get discrete codes for given inputs."""
        z_e = self.encoder(perception, goal)
        _, info = self.quantizer(z_e)
        return info['encodings']

    def decode_codes(
        self,
        codes: torch.Tensor,
        goal: torch.Tensor,
    ) -> torch.Tensor:
        """Decode from discrete codes."""
        # Get embeddings from codes
        z_q = self.quantizer.embedding(codes)
        # Decode to trajectory
        trajectory = self.decoder(z_q, goal)
        return trajectory


def test_vqvae():
    """Unit test for VQVAE."""
    batch_size = 4
    perception_dim = 512
    goal_dim = 3
    horizon = 30

    model = VQVAE(
        perception_dim=perception_dim,
        goal_dim=goal_dim,
        latent_dim=64,
        num_embeddings=128,
        horizon=horizon,
    )

    # Test data
    perception = torch.randn(batch_size, perception_dim)
    goal = torch.randn(batch_size, goal_dim)

    # Forward pass
    trajectory, info = model(perception, goal)

    print(f"Trajectory shape: {trajectory.shape}")  # [4, 30, 2]
    print(f"VQ Loss: {info['vq_loss']:.4f}")
    print(f"Perplexity: {info['perplexity']:.2f}")
    print(f"Unique codes used: {len(torch.unique(info['encodings']))}")

    assert trajectory.shape == (batch_size, horizon, 2)
    assert info['encodings'].shape == (batch_size,)

    print("✓ VQVAE test passed!")


if __name__ == '__main__':
    test_vqvae()
