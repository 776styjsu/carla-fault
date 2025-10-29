"""Fault injection orchestration module."""

from typing import Dict, List, Optional, Any
import time

from faults.base_fault import BaseFault
from utils.logging_utils import setup_logger


class FaultInjector:
    """
    Manage and inject faults into CARLA simulation.
    """
    
    def __init__(self):
        """Initialize fault injector."""
        self.faults: Dict[str, BaseFault] = {}
        self.active_faults: List[str] = []
        self.injection_log: List[Dict[str, Any]] = []
        self.logger = setup_logger("fault_injector")
    
    def register_fault(self, fault: BaseFault) -> bool:
        """
        Register a fault for injection.
        
        Args:
            fault: Fault instance to register
            
        Returns:
            True if registered successfully, False otherwise
        """
        if fault.name in self.faults:
            self.logger.warning(f"Fault '{fault.name}' already registered")
            return False
        
        self.faults[fault.name] = fault
        self.logger.info(f"Registered fault: {fault}")
        return True
    
    def unregister_fault(self, fault_name: str) -> bool:
        """
        Unregister a fault.
        
        Args:
            fault_name: Name of the fault to unregister
            
        Returns:
            True if unregistered successfully, False otherwise
        """
        if fault_name not in self.faults:
            self.logger.warning(f"Fault '{fault_name}' not registered")
            return False
        
        # Deactivate if active
        if fault_name in self.active_faults:
            self.remove_fault(fault_name, None)
        
        del self.faults[fault_name]
        self.logger.info(f"Unregistered fault '{fault_name}'")
        return True
    
    def inject_fault(self, fault_name: str, target: Any, delay: float = 0.0) -> bool:
        """
        Inject a fault into a target.
        
        Args:
            fault_name: Name of the fault to inject
            target: Target object to inject fault into
            delay: Delay before injection in seconds
            
        Returns:
            True if injected successfully, False otherwise
        """
        if fault_name not in self.faults:
            self.logger.error(f"Fault '{fault_name}' not registered")
            return False
        
        if delay > 0:
            self.logger.info(f"Waiting {delay}s before injecting '{fault_name}'")
            time.sleep(delay)
        
        fault = self.faults[fault_name]
        
        try:
            fault.inject(target)
            self.active_faults.append(fault_name)
            
            # Log injection
            self.injection_log.append({
                'timestamp': time.time(),
                'action': 'inject',
                'fault_name': fault_name,
                'target': str(target),
                'severity': fault.severity
            })
            
            self.logger.info(f"Injected fault '{fault_name}' into {target}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to inject fault '{fault_name}': {e}")
            return False
    
    def remove_fault(self, fault_name: str, target: Optional[Any] = None) -> bool:
        """
        Remove a fault from a target.
        
        Args:
            fault_name: Name of the fault to remove
            target: Target object to remove fault from (can be None for cleanup)
            
        Returns:
            True if removed successfully, False otherwise
        """
        if fault_name not in self.faults:
            self.logger.error(f"Fault '{fault_name}' not registered")
            return False
        
        if fault_name not in self.active_faults:
            self.logger.warning(f"Fault '{fault_name}' is not active")
            return False
        
        fault = self.faults[fault_name]
        
        try:
            if target is not None:
                fault.remove(target)
            else:
                # Just deactivate without calling remove if no target provided
                fault.deactivate()
            
            self.active_faults.remove(fault_name)
            
            # Log removal
            self.injection_log.append({
                'timestamp': time.time(),
                'action': 'remove',
                'fault_name': fault_name,
                'target': str(target)
            })
            
            self.logger.info(f"Removed fault '{fault_name}' from {target}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to remove fault '{fault_name}': {e}")
            return False
    
    def inject_scenario(
        self,
        scenario: List[Dict[str, Any]],
        targets: Dict[str, Any]
    ) -> bool:
        """
        Inject a sequence of faults according to a scenario.
        
        Args:
            scenario: List of fault injection events
                Each event should have: fault_name, target_name, delay (optional)
            targets: Dictionary mapping target names to target objects
            
        Returns:
            True if scenario executed successfully, False otherwise
        """
        self.logger.info(f"Executing fault scenario with {len(scenario)} events")
        
        for event in scenario:
            fault_name = event.get('fault_name')
            target_name = event.get('target_name')
            delay = event.get('delay', 0.0)
            
            if fault_name is None or target_name is None:
                self.logger.error(f"Invalid scenario event: {event}")
                return False
            
            if target_name not in targets:
                self.logger.error(f"Target '{target_name}' not found")
                return False
            
            target = targets[target_name]
            
            if not self.inject_fault(fault_name, target, delay):
                self.logger.error(f"Failed to execute scenario event: {event}")
                return False
        
        self.logger.info("Fault scenario executed successfully")
        return True
    
    def get_active_faults(self) -> List[str]:
        """Get list of currently active faults."""
        return self.active_faults.copy()
    
    def get_fault(self, fault_name: str) -> Optional[BaseFault]:
        """
        Get a registered fault by name.
        
        Args:
            fault_name: Name of the fault
            
        Returns:
            Fault instance or None if not found
        """
        return self.faults.get(fault_name)
    
    def list_faults(self) -> List[str]:
        """
        List all registered faults.
        
        Returns:
            List of fault names
        """
        return list(self.faults.keys())
    
    def get_injection_log(self) -> List[Dict[str, Any]]:
        """
        Get the fault injection log.
        
        Returns:
            List of injection events
        """
        return self.injection_log.copy()
