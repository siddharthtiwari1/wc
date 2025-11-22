"""
Trajectory decoder module.

This is typically part of VQVAE, but separated for modularity.
"""

import torch
import torch.nn as nn


class TrajectoryDecoder(nn.Module):
    """
    Decode latent codes to trajectories.

    Can use different architectures (MLP, RNN, Transformer).
    """

    def __init__(
        self,
        latent_dim: int = 64,
        goal_dim: int = 3,
        hidden_dims: list = [128, 256, 512],
        horizon: int = 30,
        output_dim: int = 2,
        decoder_type: str = 'mlp',  # 'mlp', 'rnn', 'transformer'
    ):
        super().__init__()

        self.horizon = horizon
        self.output_dim = output_dim
        self.decoder_type = decoder_type

        if decoder_type == 'mlp':
            self._build_mlp_decoder(latent_dim, goal_dim, hidden_dims)
        elif decoder_type == 'rnn':
            self._build_rnn_decoder(latent_dim, goal_dim, hidden_dims)
        elif decoder_type == 'transformer':
            self._build_transformer_decoder(latent_dim, goal_dim, hidden_dims)
        else:
            raise ValueError(f"Unknown decoder type: {decoder_type}")

    def _build_mlp_decoder(self, latent_dim, goal_dim, hidden_dims):
        """Build MLP-based decoder (all points at once)."""
        layers = []
        prev_dim = latent_dim + goal_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(inplace=True),
                nn.LayerNorm(hidden_dim),
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, self.horizon * self.output_dim))
        self.decoder = nn.Sequential(*layers)

    def _build_rnn_decoder(self, latent_dim, goal_dim, hidden_dims):
        """Build RNN-based decoder (autoregressive)."""
        self.initial_proj = nn.Linear(latent_dim + goal_dim, hidden_dims[0])

        self.rnn = nn.LSTM(
            input_size=self.output_dim + hidden_dims[0],
            hidden_size=hidden_dims[-1],
            num_layers=len(hidden_dims),
            batch_first=True,
        )

        self.output_proj = nn.Linear(hidden_dims[-1], self.output_dim)

    def _build_transformer_decoder(self, latent_dim, goal_dim, hidden_dims):
        """Build Transformer-based decoder."""
        hidden_dim = hidden_dims[-1]

        self.input_proj = nn.Linear(latent_dim + goal_dim, hidden_dim)

        decoder_layer = nn.TransformerDecoderLayer(
            d_model=hidden_dim,
            nhead=8,
            dim_feedforward=hidden_dim * 4,
            dropout=0.1,
            batch_first=True,
        )

        self.transformer = nn.TransformerDecoder(
            decoder_layer,
            num_layers=4,
        )

        # Positional encoding for time steps
        self.pos_encoding = nn.Parameter(
            torch.randn(1, self.horizon, hidden_dim) * 0.02
        )

        self.output_proj = nn.Linear(hidden_dim, self.output_dim)

    def forward(self, latent: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        """
        Decode latent to trajectory.

        Args:
            latent: [B, latent_dim]
            goal: [B, goal_dim]

        Returns:
            trajectory: [B, horizon, output_dim]
        """
        if self.decoder_type == 'mlp':
            return self._forward_mlp(latent, goal)
        elif self.decoder_type == 'rnn':
            return self._forward_rnn(latent, goal)
        elif self.decoder_type == 'transformer':
            return self._forward_transformer(latent, goal)

    def _forward_mlp(self, latent, goal):
        """MLP forward pass."""
        x = torch.cat([latent, goal], dim=-1)
        output = self.decoder(x)
        trajectory = output.view(-1, self.horizon, self.output_dim)
        return trajectory

    def _forward_rnn(self, latent, goal):
        """RNN forward pass (autoregressive)."""
        batch_size = latent.shape[0]
        device = latent.device

        # Initial hidden state from latent and goal
        h0 = self.initial_proj(torch.cat([latent, goal], dim=-1))
        h0 = h0.unsqueeze(0).repeat(self.rnn.num_layers, 1, 1)
        c0 = torch.zeros_like(h0)

        # Start from origin
        current_pos = torch.zeros(batch_size, 1, self.output_dim, device=device)

        trajectory = []

        for t in range(self.horizon):
            # Condition on latent
            latent_rep = h0[0].unsqueeze(1)  # [B, 1, hidden]
            rnn_input = torch.cat([current_pos, latent_rep], dim=-1)

            # RNN step
            output, (h0, c0) = self.rnn(rnn_input, (h0, c0))

            # Project to position
            next_pos = self.output_proj(output)  # [B, 1, output_dim]

            trajectory.append(next_pos)
            current_pos = next_pos

        trajectory = torch.cat(trajectory, dim=1)  # [B, horizon, output_dim]
        return trajectory

    def _forward_transformer(self, latent, goal):
        """Transformer forward pass."""
        batch_size = latent.shape[0]
        device = latent.device

        # Memory from latent and goal
        memory = self.input_proj(torch.cat([latent, goal], dim=-1))
        memory = memory.unsqueeze(1)  # [B, 1, hidden]

        # Query positions with positional encoding
        queries = self.pos_encoding.repeat(batch_size, 1, 1)  # [B, horizon, hidden]

        # Transformer decode
        output = self.transformer(queries, memory)  # [B, horizon, hidden]

        # Project to trajectory
        trajectory = self.output_proj(output)  # [B, horizon, output_dim]

        return trajectory


if __name__ == '__main__':
    # Test all decoder types
    for decoder_type in ['mlp', 'rnn', 'transformer']:
        decoder = TrajectoryDecoder(
            latent_dim=64,
            goal_dim=3,
            hidden_dims=[128, 256],
            horizon=30,
            output_dim=2,
            decoder_type=decoder_type,
        )

        latent = torch.randn(4, 64)
        goal = torch.randn(4, 3)

        trajectory = decoder(latent, goal)

        print(f"{decoder_type} decoder output shape: {trajectory.shape}")
        assert trajectory.shape == (4, 30, 2)

    print("✓ All decoder tests passed!")
