"""
PixelCNN prior for autoregressive code generation.

Used to model the distribution of discrete codes from VQVAE.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Optional


class MaskedConv1d(nn.Conv1d):
    """1D convolution with causal masking (for autoregressive generation)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.register_buffer('mask', torch.zeros_like(self.weight))
        self._update_mask()

    def _update_mask(self):
        """Create causal mask."""
        kernel_size = self.kernel_size[0]
        # Allow current and previous positions
        self.mask[:, :, :kernel_size // 2 + 1] = 1.0

    def forward(self, x):
        self.weight.data *= self.mask
        return super().forward(x)


class GatedResidualBlock(nn.Module):
    """
    Gated residual block for PixelCNN.

    Uses gated activation units for improved gradient flow.
    """

    def __init__(
        self,
        channels: int,
        kernel_size: int = 3,
        dropout: float = 0.1,
    ):
        super().__init__()

        padding = kernel_size // 2

        self.conv1 = MaskedConv1d(
            channels, 2 * channels,
            kernel_size=kernel_size,
            padding=padding,
        )

        self.dropout = nn.Dropout(dropout)

        self.conv2 = nn.Conv1d(channels, channels, kernel_size=1)

        self.ln = nn.LayerNorm(channels)

    def forward(self, x):
        """
        Args:
            x: [B, C, L]
        Returns:
            [B, C, L]
        """
        residual = x

        # Gated activation
        out = self.conv1(x)
        out_a, out_b = out.chunk(2, dim=1)
        out = torch.tanh(out_a) * torch.sigmoid(out_b)

        # Residual connection
        out = self.dropout(out)
        out = self.conv2(out)

        out = out + residual

        # Layer norm
        out = out.transpose(1, 2)  # [B, L, C]
        out = self.ln(out)
        out = out.transpose(1, 2)  # [B, C, L]

        return out


class PixelCNN(nn.Module):
    """
    PixelCNN for modeling distribution over VQVAE codes.

    Autoregressively predicts next code given previous codes.
    """

    def __init__(
        self,
        num_embeddings: int = 512,
        num_layers: int = 6,
        hidden_channels: int = 128,
        kernel_size: int = 5,
        dropout: float = 0.1,
        condition_dim: Optional[int] = None,  # For conditioning on goal
    ):
        super().__init__()

        self.num_embeddings = num_embeddings
        self.condition_dim = condition_dim

        # Embedding for discrete codes
        self.code_embedding = nn.Embedding(num_embeddings, hidden_channels)

        # Conditioning network
        if condition_dim is not None:
            self.condition_proj = nn.Linear(condition_dim, hidden_channels)

        # Initial convolution
        self.input_conv = nn.Conv1d(
            hidden_channels,
            hidden_channels,
            kernel_size=1,
        )

        # Residual blocks
        self.blocks = nn.ModuleList([
            GatedResidualBlock(
                hidden_channels,
                kernel_size=kernel_size,
                dropout=dropout,
            )
            for _ in range(num_layers)
        ])

        # Output projection
        self.output_conv = nn.Sequential(
            nn.Conv1d(hidden_channels, hidden_channels, kernel_size=1),
            nn.ReLU(inplace=True),
            nn.Conv1d(hidden_channels, num_embeddings, kernel_size=1),
        )

    def forward(
        self,
        codes: torch.Tensor,
        condition: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """
        Predict distribution over next codes.

        Args:
            codes: Discrete codes [B, L]
            condition: Optional conditioning vector [B, condition_dim]

        Returns:
            logits: [B, num_embeddings, L]
        """
        # Embed codes
        x = self.code_embedding(codes)  # [B, L, C]
        x = x.transpose(1, 2)  # [B, C, L]

        # Add conditioning
        if condition is not None and self.condition_dim is not None:
            cond = self.condition_proj(condition)  # [B, C]
            cond = cond.unsqueeze(2)  # [B, C, 1]
            x = x + cond

        # Process
        x = self.input_conv(x)

        for block in self.blocks:
            x = block(x)

        # Output logits
        logits = self.output_conv(x)  # [B, num_embeddings, L]

        return logits

    def sample(
        self,
        batch_size: int = 1,
        length: int = 1,
        condition: Optional[torch.Tensor] = None,
        temperature: float = 1.0,
        device: str = 'cpu',
    ) -> torch.Tensor:
        """
        Autoregressively sample codes.

        Args:
            batch_size: Number of samples
            length: Sequence length
            condition: Optional conditioning [B, condition_dim]
            temperature: Sampling temperature
            device: torch device

        Returns:
            codes: [B, length]
        """
        self.eval()

        # Start with random code
        codes = torch.randint(
            0, self.num_embeddings,
            (batch_size, 1),
            device=device,
        )

        with torch.no_grad():
            for _ in range(length - 1):
                # Get logits for current sequence
                logits = self(codes, condition)  # [B, K, L]

                # Get logits for last position
                next_logits = logits[:, :, -1] / temperature  # [B, K]

                # Sample
                probs = F.softmax(next_logits, dim=-1)
                next_code = torch.multinomial(probs, num_samples=1)  # [B, 1]

                # Append
                codes = torch.cat([codes, next_code], dim=1)

        return codes


def test_pixelcnn():
    """Unit test for PixelCNN."""
    batch_size = 4
    seq_length = 5
    num_embeddings = 128
    condition_dim = 3

    model = PixelCNN(
        num_embeddings=num_embeddings,
        num_layers=4,
        hidden_channels=64,
        condition_dim=condition_dim,
    )

    # Test data
    codes = torch.randint(0, num_embeddings, (batch_size, seq_length))
    condition = torch.randn(batch_size, condition_dim)

    # Forward pass
    logits = model(codes, condition)

    print(f"Logits shape: {logits.shape}")  # [4, 128, 5]
    assert logits.shape == (batch_size, num_embeddings, seq_length)

    # Test sampling
    samples = model.sample(
        batch_size=2,
        length=10,
        condition=condition[:2],
        temperature=0.8,
        device='cpu',
    )

    print(f"Samples shape: {samples.shape}")  # [2, 10]
    assert samples.shape == (2, 10)
    assert samples.max() < num_embeddings
    assert samples.min() >= 0

    print("✓ PixelCNN test passed!")


if __name__ == '__main__':
    test_pixelcnn()
