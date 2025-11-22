"""
1D U-Net for trajectory diffusion model.
"""

import torch
import torch.nn as nn
import math


class SinusoidalPosEmb(nn.Module):
    """Sinusoidal positional embeddings for timestep."""

    def __init__(self, dim):
        super().__init__()
        self.dim = dim

    def forward(self, timesteps):
        device = timesteps.device
        half_dim = self.dim // 2
        embeddings = math.log(10000) / (half_dim - 1)
        embeddings = torch.exp(torch.arange(half_dim, device=device) * -embeddings)
        embeddings = timesteps[:, None] * embeddings[None, :]
        embeddings = torch.cat((embeddings.sin(), embeddings.cos()), dim=-1)
        return embeddings


class UNet1D(nn.Module):
    """
    1D U-Net for denoising trajectories.

    Takes noised trajectory and predicts noise.
    """

    def __init__(
        self,
        in_channels: int = 2,
        out_channels: int = 2,
        time_emb_dim: int = 128,
        condition_dim: int = 512,
        model_channels: int = 128,
        num_layers: int = 4,
    ):
        super().__init__()

        self.time_mlp = nn.Sequential(
            SinusoidalPosEmb(time_emb_dim),
            nn.Linear(time_emb_dim, time_emb_dim * 4),
            nn.GELU(),
            nn.Linear(time_emb_dim * 4, time_emb_dim),
        )

        self.condition_proj = nn.Linear(condition_dim, model_channels)

        # Input conv
        self.input_conv = nn.Conv1d(in_channels, model_channels, kernel_size=3, padding=1)

        # Down blocks
        self.down_blocks = nn.ModuleList([
            self._make_down_block(model_channels, model_channels * 2),
            self._make_down_block(model_channels * 2, model_channels * 4),
        ])

        # Bottleneck
        self.bottleneck = nn.Sequential(
            nn.Conv1d(model_channels * 4, model_channels * 4, 3, padding=1),
            nn.GroupNorm(8, model_channels * 4),
            nn.GELU(),
        )

        # Up blocks
        self.up_blocks = nn.ModuleList([
            self._make_up_block(model_channels * 8, model_channels * 2),  # concat from down
            self._make_up_block(model_channels * 4, model_channels),
        ])

        # Output
        self.output_conv = nn.Sequential(
            nn.Conv1d(model_channels * 2, model_channels, 3, padding=1),
            nn.GroupNorm(8, model_channels),
            nn.GELU(),
            nn.Conv1d(model_channels, out_channels, 1),
        )

    def _make_down_block(self, in_ch, out_ch):
        return nn.Sequential(
            nn.Conv1d(in_ch, out_ch, 3, padding=1, stride=2),
            nn.GroupNorm(8, out_ch),
            nn.GELU(),
        )

    def _make_up_block(self, in_ch, out_ch):
        return nn.Sequential(
            nn.ConvTranspose1d(in_ch, out_ch, 4, stride=2, padding=1),
            nn.GroupNorm(8, out_ch),
            nn.GELU(),
        )

    def forward(self, x, t, condition):
        """
        Args:
            x: [B, T, D] trajectory
            t: [B] timesteps
            condition: [B, C] conditioning

        Returns:
            noise: [B, T, D] predicted noise
        """
        # Transpose for conv1d: [B, D, T]
        x = x.transpose(1, 2)

        # Time embedding
        t_emb = self.time_mlp(t)  # [B, time_emb_dim]

        # Condition embedding
        cond_emb = self.condition_proj(condition)  # [B, model_channels]

        # Input
        h = self.input_conv(x)

        # Add condition
        h = h + cond_emb[:, :, None]

        # Downsampling
        skip_connections = [h]
        for down in self.down_blocks:
            h = down(h)
            skip_connections.append(h)

        # Bottleneck
        h = self.bottleneck(h)

        # Upsampling with skip connections
        for up in self.up_blocks:
            skip = skip_connections.pop()
            h = torch.cat([h, skip], dim=1)
            h = up(h)

        # Output
        skip = skip_connections.pop()
        h = torch.cat([h, skip], dim=1)
        h = self.output_conv(h)

        # Transpose back: [B, T, D]
        h = h.transpose(1, 2)

        return h


if __name__ == '__main__':
    model = UNet1D(
        in_channels=2,
        out_channels=2,
        time_emb_dim=128,
        condition_dim=512,
        model_channels=128,
    )

    x = torch.randn(4, 30, 2)  # batch, time, dim
    t = torch.randint(0, 1000, (4,))
    cond = torch.randn(4, 512)

    out = model(x, t, cond)

    print(f"Output shape: {out.shape}")
    assert out.shape == x.shape

    print("✓ UNet1D test passed!")
