"""Base fault class for CARLA fault injection."""

from abc import ABC, abstractmethod
from typing import Any, Optional
import logging


class BaseFault(ABC):
    """
    Abstract base class for all fault types.
    
    This class defines the interface that all fault implementations must follow.
    """
    
    def __init__(self, name: str, severity: float = 1.0):
        """
        Initialize a fault.
        
        Args:
            name: Name of the fault
            severity: Severity level of the fault (0.0 to 1.0)
        """
        self.name = name
        self.severity = max(0.0, min(1.0, severity))  # Clamp to [0, 1]
        self.is_active = False
        self.logger = logging.getLogger(f"fault.{name}")
    
    @abstractmethod
    def inject(self, target: Any) -> None:
        """
        Inject the fault into the target.
        
        Args:
            target: The target object to inject the fault into
        """
        pass
    
    @abstractmethod
    def remove(self, target: Any) -> None:
        """
        Remove the fault from the target.
        
        Args:
            target: The target object to remove the fault from
        """
        pass
    
    def activate(self) -> None:
        """Activate the fault."""
        self.is_active = True
        self.logger.info(f"Fault '{self.name}' activated with severity {self.severity}")
    
    def deactivate(self) -> None:
        """Deactivate the fault."""
        self.is_active = False
        self.logger.info(f"Fault '{self.name}' deactivated")
    
    def __repr__(self) -> str:
        """String representation of the fault."""
        status = "active" if self.is_active else "inactive"
        return f"{self.__class__.__name__}(name='{self.name}', severity={self.severity}, status={status})"
