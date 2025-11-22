"""
Perception encoder for processing sensory inputs.

Converts LaserScan or PointCloud2 data into compact feature vectors.
"""

import torch
import torch.nn as nn
import numpy as np
from typing import Optional


class PerceptionEncoder(nn.Module):
    """
    Encode perception data (laser scans, point clouds) into features.

    Supports both 2D laser scans and 3D point clouds.
    """

    def __init__(
        self,
        input_type: str = 'laserscan',  # 'laserscan' or 'pointcloud'
        input_dim: int = 360,            # Number of laser rays or point features
        output_dim: int = 512,           # Feature dimension
        hidden_dims: list = [256, 512, 512],
        use_attention: bool = True,
    ):
        super().__init__()

        self.input_type = input_type
        self.input_dim = input_dim
        self.output_dim = output_dim

        # Input processing
        if input_type == 'laserscan':
            # For laser scans, use 1D CNN
            self.input_proc = nn.Sequential(
                nn.Conv1d(1, 32, kernel_size=5, padding=2),
                nn.ReLU(inplace=True),
                nn.MaxPool1d(2),
                nn.Conv1d(32, 64, kernel_size=5, padding=2),
                nn.ReLU(inplace=True),
                nn.MaxPool1d(2),
                nn.Conv1d(64, 128, kernel_size=5, padding=2),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool1d(1),
            )
            mlp_input_dim = 128

        else:  # pointcloud
            # For point clouds, use PointNet-style architecture
            self.input_proc = nn.Sequential(
                nn.Linear(input_dim, 64),
                nn.ReLU(inplace=True),
                nn.Linear(64, 128),
                nn.ReLU(inplace=True),
                nn.Linear(128, 256),
            )
            mlp_input_dim = 256

        # Attention layer
        if use_attention:
            self.attention = nn.MultiheadAttention(
                embed_dim=mlp_input_dim,
                num_heads=4,
                batch_first=True,
            )
        else:
            self.attention = None

        # MLP layers
        layers = []
        prev_dim = mlp_input_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(inplace=True),
                nn.LayerNorm(hidden_dim),
                nn.Dropout(0.1),
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, output_dim))

        self.mlp = nn.Sequential(*layers)

    def forward(self, perception_input: torch.Tensor) -> torch.Tensor:
        """
        Encode perception data.

        Args:
            perception_input:
                For laserscan: [B, num_rays] or [B, 1, num_rays]
                For pointcloud: [B, num_points, features]

        Returns:
            features: [B, output_dim]
        """
        if self.input_type == 'laserscan':
            # Ensure correct shape
            if perception_input.ndim == 2:
                perception_input = perception_input.unsqueeze(1)  # [B, 1, num_rays]

            # CNN processing
            x = self.input_proc(perception_input)  # [B, 128, 1]
            x = x.squeeze(-1)  # [B, 128]

        else:  # pointcloud
            # Process each point
            x = self.input_proc(perception_input)  # [B, num_points, 256]

            # Max pooling over points (PointNet-style)
            if self.attention is not None:
                # Use attention before pooling
                x_att, _ = self.attention(x, x, x)  # [B, num_points, 256]
                x = torch.max(x_att, dim=1)[0]  # [B, 256]
            else:
                x = torch.max(x, dim=1)[0]  # [B, 256]

        # MLP to output features
        features = self.mlp(x)  # [B, output_dim]

        return features


class BEVEncoder(nn.Module):
    """
    Bird's Eye View (BEV) encoder for spatial perception.

    Converts point cloud to BEV grid and uses 2D CNN.
    """

    def __init__(
        self,
        grid_size: int = 64,
        grid_range: float = 10.0,  # meters
        output_dim: int = 512,
    ):
        super().__init__()

        self.grid_size = grid_size
        self.grid_range = grid_range
        self.resolution = 2 * grid_range / grid_size

        # 2D CNN for BEV grid
        self.cnn = nn.Sequential(
            # Input: [B, 1, H, W]
            nn.Conv2d(1, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),  # -> [B, 32, H/2, W/2]

            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),  # -> [B, 64, H/4, W/4]

            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),  # -> [B, 128, H/8, W/8]

            nn.AdaptiveAvgPool2d(1),  # -> [B, 128, 1, 1]
        )

        self.mlp = nn.Sequential(
            nn.Linear(128, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, output_dim),
        )

    def points_to_bev(self, points: torch.Tensor) -> torch.Tensor:
        """
        Convert point cloud to BEV occupancy grid.

        Args:
            points: [B, N, 2] (x, y coordinates)

        Returns:
            grid: [B, 1, grid_size, grid_size]
        """
        batch_size = points.shape[0]
        device = points.device

        # Initialize grid
        grid = torch.zeros(
            batch_size, 1, self.grid_size, self.grid_size,
            device=device,
        )

        # Convert coordinates to grid indices
        grid_coords = (points + self.grid_range) / self.resolution
        grid_coords = grid_coords.long()

        # Clip to grid bounds
        grid_coords = torch.clamp(grid_coords, 0, self.grid_size - 1)

        # Fill grid
        for b in range(batch_size):
            grid[b, 0, grid_coords[b, :, 1], grid_coords[b, :, 0]] = 1.0

        return grid

    def forward(self, points: torch.Tensor) -> torch.Tensor:
        """
        Encode point cloud via BEV representation.

        Args:
            points: [B, N, 2]

        Returns:
            features: [B, output_dim]
        """
        # Convert to BEV grid
        bev_grid = self.points_to_bev(points)  # [B, 1, H, W]

        # CNN processing
        x = self.cnn(bev_grid)  # [B, 128, 1, 1]
        x = x.squeeze(-1).squeeze(-1)  # [B, 128]

        # MLP
        features = self.mlp(x)  # [B, output_dim]

        return features


def test_perception_encoder():
    """Unit test for perception encoder."""
    batch_size = 4

    # Test LaserScan encoder
    laser_encoder = PerceptionEncoder(
        input_type='laserscan',
        input_dim=360,
        output_dim=512,
    )

    laser_data = torch.randn(batch_size, 360)
    features = laser_encoder(laser_data)

    print(f"Laser features shape: {features.shape}")
    assert features.shape == (batch_size, 512)

    # Test PointCloud encoder
    pc_encoder = PerceptionEncoder(
        input_type='pointcloud',
        input_dim=3,
        output_dim=512,
    )

    pc_data = torch.randn(batch_size, 100, 3)  # 100 points with (x, y, z)
    features = pc_encoder(pc_data)

    print(f"PointCloud features shape: {features.shape}")
    assert features.shape == (batch_size, 512)

    # Test BEV encoder
    bev_encoder = BEVEncoder(
        grid_size=64,
        grid_range=10.0,
        output_dim=512,
    )

    points = torch.randn(batch_size, 200, 2)  # 200 points with (x, y)
    features = bev_encoder(points)

    print(f"BEV features shape: {features.shape}")
    assert features.shape == (batch_size, 512)

    print("✓ Perception encoder tests passed!")


if __name__ == '__main__':
    test_perception_encoder()
