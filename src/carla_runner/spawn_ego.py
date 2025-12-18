"""Ego vehicle spawning module."""

import random
from typing import Optional

from carla_utils.logging_utils import setup_logger


class EgoSpawner:
    """
    Spawn and manage the ego vehicle in CARLA.
    """
    
    def __init__(self, client):
        """
        Initialize ego spawner.
        
        Args:
            client: CARLA client instance
        """
        self.client = client
        self.world = client.get_world()
        self.ego_vehicle = None
        self.logger = setup_logger("ego_spawner")
    
    def spawn(
        self,
        vehicle_model: str = "vehicle.tesla.model3",
        spawn_point: Optional[int] = None
    ) -> Optional[object]:
        """
        Spawn the ego vehicle in the world.
        
        Args:
            vehicle_model: Blueprint name of the vehicle
            spawn_point: Index of spawn point (None for random)
            
        Returns:
            Spawned vehicle actor or None if failed
        """
        try:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find(vehicle_model)
            
            if vehicle_bp is None:
                self.logger.error(f"Vehicle model '{vehicle_model}' not found")
                return None
            
            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            
            if not spawn_points:
                self.logger.error("No spawn points available")
                return None
            
            # Select spawn point
            if spawn_point is None:
                spawn_transform = random.choice(spawn_points)
                self.logger.info("Using random spawn point")
            else:
                if spawn_point >= len(spawn_points):
                    self.logger.warning(
                        f"Spawn point {spawn_point} out of range, using random"
                    )
                    spawn_transform = random.choice(spawn_points)
                else:
                    spawn_transform = spawn_points[spawn_point]
                    self.logger.info(f"Using spawn point {spawn_point}")
            
            # Spawn vehicle
            self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
            self.logger.info(
                f"Spawned ego vehicle '{vehicle_model}' at "
                f"({spawn_transform.location.x:.2f}, "
                f"{spawn_transform.location.y:.2f}, "
                f"{spawn_transform.location.z:.2f})"
            )
            
            return self.ego_vehicle
            
        except Exception as e:
            self.logger.error(f"Failed to spawn ego vehicle: {e}")
            return None
    
    def destroy(self) -> bool:
        """
        Destroy the ego vehicle.
        
        Returns:
            True if destroyed successfully, False otherwise
        """
        if self.ego_vehicle is None:
            self.logger.warning("No ego vehicle to destroy")
            return False
        
        try:
            self.ego_vehicle.destroy()
            self.logger.info("Ego vehicle destroyed")
            self.ego_vehicle = None
            return True
        except Exception as e:
            self.logger.error(f"Failed to destroy ego vehicle: {e}")
            return False
    
    def get_vehicle(self):
        """Get the ego vehicle actor."""
        return self.ego_vehicle
    
    def get_transform(self):
        """Get current transform of the ego vehicle."""
        if self.ego_vehicle is None:
            return None
        return self.ego_vehicle.get_transform()
    
    def set_autopilot(self, enabled: bool = True) -> None:
        """
        Enable or disable autopilot for ego vehicle.
        
        Args:
            enabled: True to enable, False to disable
        """
        if self.ego_vehicle is None:
            self.logger.warning("No ego vehicle to set autopilot")
            return
        
        self.ego_vehicle.set_autopilot(enabled)
        status = "enabled" if enabled else "disabled"
        self.logger.info(f"Autopilot {status}")
