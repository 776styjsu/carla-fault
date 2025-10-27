"""Camera blackout fault implementation."""

from typing import Any, Optional
import numpy as np
from .base_fault import BaseFault


class CameraBlackoutFault(BaseFault):
    """
    Fault that simulates a camera blackout/failure.
    
    When injected, this fault will cause the camera to output black frames.
    """
    
    def __init__(self, name: str = "camera_blackout", severity: float = 1.0):
        """
        Initialize camera blackout fault.
        
        Args:
            name: Name of the fault (default: "camera_blackout")
            severity: Severity level (0.0 = no effect, 1.0 = complete blackout)
        """
        super().__init__(name, severity)
        self.original_callback = None
    
    def inject(self, target: Any) -> None:
        """
        Inject the camera blackout fault.
        
        Args:
            target: Camera sensor object
        """
        if self.is_active:
            self.logger.warning(f"Fault '{self.name}' is already active")
            return
        
        self.activate()
        self.logger.info(f"Injecting camera blackout fault into {target}")
        
        # Store original callback if it exists
        if hasattr(target, 'listen'):
            # For CARLA sensors, we would wrap the callback here
            # This is a simplified implementation
            pass
    
    def remove(self, target: Any) -> None:
        """
        Remove the camera blackout fault.
        
        Args:
            target: Camera sensor object
        """
        if not self.is_active:
            self.logger.warning(f"Fault '{self.name}' is not active")
            return
        
        self.deactivate()
        self.logger.info(f"Removing camera blackout fault from {target}")
        
        # Restore original callback if it was stored
        if self.original_callback is not None:
            # Restore the original callback
            self.original_callback = None
    
    def apply_to_image(self, image: np.ndarray) -> np.ndarray:
        """
        Apply blackout effect to an image.
        
        Args:
            image: Input image as numpy array
            
        Returns:
            Modified image with blackout applied
        """
        if not self.is_active:
            return image
        
        # Create a black image of the same shape
        black_image = np.zeros_like(image)
        
        # Blend based on severity (0 = original, 1 = completely black)
        result = image * (1 - self.severity) + black_image * self.severity
        
        return result.astype(image.dtype)
