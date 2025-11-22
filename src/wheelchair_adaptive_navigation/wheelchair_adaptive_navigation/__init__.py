"""
Adaptive Crowd Navigation with Diffusion Models

Novel approach featuring:
- Denoising Diffusion Probabilistic Models for trajectory generation
- Uncertainty-aware planning with risk quantification
- Social group modeling and awareness
- Hierarchical planning (global/local/reflex)
"""

__version__ = '0.1.0'
__author__ = 'Siddharth Tiwari'
__email__ = 's24035@students.iitmandi.ac.in'

from . import models
from . import planning
from . import perception
from . import ros2
from . import training

__all__ = ['models', 'planning', 'perception', 'ros2', 'training']
