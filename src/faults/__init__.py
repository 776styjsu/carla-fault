"""Faults package for CARLA fault injection."""

from .base_fault import BaseFault
from .camera_blackout import CameraBlackoutFault

__all__ = ['BaseFault', 'CameraBlackoutFault']
