"""
CrowdSurfer-based Dense Crowd Navigation for Wheelchairs

This package implements the CrowdSurfer approach (ICRA 2025) for ROS2 Jazzy,
combining Vector Quantized VAE with sampling-based optimization for
navigation in dense crowds.
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
