"""CARLA launcher module."""

import subprocess
import time
from typing import Optional
import os
import signal

from carla_utils.logging_utils import setup_logger


class CarlaLauncher:
    """
    Launch and manage CARLA simulator.
    """
    
    def __init__(
        self,
        # carla_path: str = "/opt/carla-simulator",
        carla_path: str = "/home/yutia000/Work/Research/Tools/carla-0.9.15/",
        port: int = 2000,
        quality_level: str = "Low"
    ):
        """
        Initialize CARLA launcher.
        
        Args:
            carla_path: Path to CARLA installation
            port: Port to run CARLA on (default: 2000)
            quality_level: Graphics quality level (Low, Medium, High, Epic)
        """
        self.carla_path = carla_path
        self.port = port
        self.quality_level = quality_level
        self.process: Optional[subprocess.Popen] = None
        self.logger = setup_logger("carla_launcher")
    
    def launch(self, timeout: int = 30) -> bool:
        """
        Launch CARLA simulator.
        
        Args:
            timeout: Seconds to wait for CARLA to start
            
        Returns:
            True if launched successfully, False otherwise
        """
        carla_exec = os.path.join(self.carla_path, "CarlaUE4.sh")
        
        if not os.path.exists(carla_exec):
            self.logger.error(f"CARLA executable not found at {carla_exec}")
            return False
        
        cmd = [
            carla_exec,
            # "-RenderOffScreen",
            f"-carla-rpc-port={self.port}",
            f"-quality-level={self.quality_level}"
        ]
        
        try:
            self.logger.info(f"Launching CARLA: {' '.join(cmd)}")
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait for CARLA to start
            self.logger.info(f"Waiting {timeout}s for CARLA to start...")
            time.sleep(timeout)
            
            if self.process.poll() is not None:
                self.logger.error("CARLA process terminated unexpectedly")
                return False
            
            self.logger.info("CARLA launched successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to launch CARLA: {e}")
            return False
    
    def shutdown(self) -> None:
        """Shutdown CARLA simulator."""
        if self.process is None:
            self.logger.warning("No CARLA process to shutdown")
            return
        
        try:
            self.logger.info("Shutting down CARLA...")
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=10)
            self.logger.info("CARLA shutdown successfully")
        except Exception as e:
            self.logger.error(f"Error shutting down CARLA: {e}")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except:
                pass
        finally:
            self.process = None
    
    def is_running(self) -> bool:
        """Check if CARLA is running."""
        return self.process is not None and self.process.poll() is None
