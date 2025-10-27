"""Sensor manager module."""

import yaml
import os
from typing import Dict, List, Optional, Any, Callable
import logging

from ..utils.logging_utils import setup_logger


class SensorManager:
    """
    Manage sensors attached to the ego vehicle.
    """
    
    def __init__(self, client, ego_vehicle):
        """
        Initialize sensor manager.
        
        Args:
            client: CARLA client instance
            ego_vehicle: The ego vehicle to attach sensors to
        """
        self.client = client
        self.world = client.get_world()
        self.ego_vehicle = ego_vehicle
        self.sensors: Dict[str, Any] = {}
        self.logger = setup_logger("sensor_manager")
    
    def load_config(self, config_path: str) -> Dict:
        """
        Load sensor configuration from YAML file.
        
        Args:
            config_path: Path to sensor configuration file
            
        Returns:
            Configuration dictionary
        """
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.logger.info(f"Loaded sensor config from {config_path}")
            return config
        except Exception as e:
            self.logger.error(f"Failed to load config: {e}")
            return {}
    
    def spawn_sensors(self, config: Dict) -> bool:
        """
        Spawn sensors from configuration.
        
        Args:
            config: Sensor configuration dictionary
            
        Returns:
            True if all sensors spawned successfully, False otherwise
        """
        if 'sensors' not in config:
            self.logger.error("No sensors defined in config")
            return False
        
        blueprint_library = self.world.get_blueprint_library()
        success = True
        
        for sensor_name, sensor_config in config['sensors'].items():
            try:
                # Get sensor blueprint
                sensor_type = sensor_config.get('type')
                sensor_bp = blueprint_library.find(sensor_type)
                
                if sensor_bp is None:
                    self.logger.error(f"Sensor type '{sensor_type}' not found")
                    success = False
                    continue
                
                # Set attributes
                attributes = sensor_config.get('attributes', {})
                for attr_name, attr_value in attributes.items():
                    if sensor_bp.has_attribute(attr_name):
                        sensor_bp.set_attribute(attr_name, str(attr_value))
                
                # Get spawn transform relative to vehicle
                spawn_point = sensor_config.get('spawn_point', {})
                import carla
                transform = carla.Transform(
                    carla.Location(
                        x=spawn_point.get('x', 0.0),
                        y=spawn_point.get('y', 0.0),
                        z=spawn_point.get('z', 0.0)
                    ),
                    carla.Rotation(
                        pitch=spawn_point.get('pitch', 0.0),
                        yaw=spawn_point.get('yaw', 0.0),
                        roll=spawn_point.get('roll', 0.0)
                    )
                )
                
                # Spawn sensor
                sensor = self.world.spawn_actor(
                    sensor_bp,
                    transform,
                    attach_to=self.ego_vehicle
                )
                
                self.sensors[sensor_name] = sensor
                self.logger.info(f"Spawned sensor '{sensor_name}' ({sensor_type})")
                
            except Exception as e:
                self.logger.error(f"Failed to spawn sensor '{sensor_name}': {e}")
                success = False
        
        return success
    
    def attach_callback(self, sensor_name: str, callback: Callable) -> bool:
        """
        Attach a callback to a sensor.
        
        Args:
            sensor_name: Name of the sensor
            callback: Callback function to attach
            
        Returns:
            True if callback attached successfully, False otherwise
        """
        if sensor_name not in self.sensors:
            self.logger.error(f"Sensor '{sensor_name}' not found")
            return False
        
        try:
            sensor = self.sensors[sensor_name]
            sensor.listen(callback)
            self.logger.info(f"Attached callback to sensor '{sensor_name}'")
            return True
        except Exception as e:
            self.logger.error(f"Failed to attach callback: {e}")
            return False
    
    def destroy_sensors(self) -> None:
        """Destroy all managed sensors."""
        for sensor_name, sensor in self.sensors.items():
            try:
                sensor.stop()
                sensor.destroy()
                self.logger.info(f"Destroyed sensor '{sensor_name}'")
            except Exception as e:
                self.logger.error(f"Failed to destroy sensor '{sensor_name}': {e}")
        
        self.sensors.clear()
    
    def get_sensor(self, sensor_name: str) -> Optional[Any]:
        """
        Get a sensor by name.
        
        Args:
            sensor_name: Name of the sensor
            
        Returns:
            Sensor actor or None if not found
        """
        return self.sensors.get(sensor_name)
    
    def list_sensors(self) -> List[str]:
        """
        List all sensor names.
        
        Returns:
            List of sensor names
        """
        return list(self.sensors.keys())
