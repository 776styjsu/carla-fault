"""Data recorder module."""

import os
import time
from typing import Optional, Dict, Any
import json

from utils.logging_utils import setup_logger


class DataRecorder:
    """
    Record sensor data and simulation events.
    """
    
    def __init__(self, output_dir: str = "data"):
        """
        Initialize data recorder.
        
        Args:
            output_dir: Directory to save recorded data
        """
        self.output_dir = output_dir
        self.recording = False
        self.start_time: Optional[float] = None
        self.frame_count = 0
        self.metadata: Dict[str, Any] = {}
        self.logger = setup_logger("recorder")
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
    
    def start_recording(self, session_name: Optional[str] = None) -> str:
        """
        Start recording session.
        
        Args:
            session_name: Name of the recording session (auto-generated if None)
            
        Returns:
            Session directory path
        """
        if self.recording:
            self.logger.warning("Recording already in progress")
            return self.session_dir
        
        # Create session directory
        if session_name is None:
            session_name = f"session_{int(time.time())}"
        
        self.session_dir = os.path.join(self.output_dir, session_name)
        os.makedirs(self.session_dir, exist_ok=True)
        
        self.recording = True
        self.start_time = time.time()
        self.frame_count = 0
        self.metadata = {
            'session_name': session_name,
            'start_time': self.start_time,
            'frames': []
        }
        
        self.logger.info(f"Started recording session: {session_name}")
        return self.session_dir
    
    def stop_recording(self) -> bool:
        """
        Stop recording session and save metadata.
        
        Returns:
            True if stopped successfully, False otherwise
        """
        if not self.recording:
            self.logger.warning("No recording in progress")
            return False
        
        self.recording = False
        end_time = time.time()
        
        self.metadata['end_time'] = end_time
        self.metadata['duration'] = end_time - self.start_time
        self.metadata['total_frames'] = self.frame_count
        
        # Save metadata
        metadata_path = os.path.join(self.session_dir, 'metadata.json')
        try:
            with open(metadata_path, 'w') as f:
                json.dump(self.metadata, f, indent=2)
            self.logger.info(f"Saved metadata to {metadata_path}")
        except Exception as e:
            self.logger.error(f"Failed to save metadata: {e}")
            return False
        
        self.logger.info(
            f"Stopped recording. Duration: {self.metadata['duration']:.2f}s, "
            f"Frames: {self.frame_count}"
        )
        return True
    
    def record_frame(self, frame_data: Dict[str, Any]) -> bool:
        """
        Record data for a single frame.
        
        Args:
            frame_data: Dictionary containing frame data
            
        Returns:
            True if recorded successfully, False otherwise
        """
        if not self.recording:
            self.logger.warning("Not recording, call start_recording() first")
            return False
        
        try:
            timestamp = time.time() - self.start_time
            frame_data['timestamp'] = timestamp
            frame_data['frame_number'] = self.frame_count
            
            self.metadata['frames'].append(frame_data)
            self.frame_count += 1
            
            return True
        except Exception as e:
            self.logger.error(f"Failed to record frame: {e}")
            return False
    
    def save_sensor_data(
        self,
        sensor_name: str,
        data: Any,
        frame_number: Optional[int] = None
    ) -> Optional[str]:
        """
        Save sensor data to file.
        
        Args:
            sensor_name: Name of the sensor
            data: Sensor data to save
            frame_number: Frame number (uses current if None)
            
        Returns:
            Path to saved file or None if failed
        """
        if not self.recording:
            self.logger.warning("Not recording, call start_recording() first")
            return None
        
        if frame_number is None:
            frame_number = self.frame_count
        
        # Create sensor directory
        sensor_dir = os.path.join(self.session_dir, sensor_name)
        os.makedirs(sensor_dir, exist_ok=True)
        
        # Save data based on type
        filename = f"{frame_number:06d}"
        
        try:
            # This is a simplified implementation
            # In practice, you would handle different data types
            # (images, point clouds, etc.) appropriately
            if hasattr(data, 'save_to_disk'):
                # CARLA sensor data
                filepath = os.path.join(sensor_dir, filename)
                data.save_to_disk(filepath)
            else:
                # Generic data
                filepath = os.path.join(sensor_dir, f"{filename}.json")
                with open(filepath, 'w') as f:
                    json.dump(str(data), f)
            
            return filepath
        except Exception as e:
            self.logger.error(f"Failed to save sensor data: {e}")
            return None
    
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self.recording
    
    def get_frame_count(self) -> int:
        """Get current frame count."""
        return self.frame_count
