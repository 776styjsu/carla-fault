"""Tire pressure imbalance fault implementation w/ Physics Integration."""

from typing import Any
from .base_fault import BaseFault


class TirePressureImbalanceFault(BaseFault):
    """
    Simulates an under-inflated tire using BOTH physics changes and
    control distortion.

    Physical effects (via CARLA physics API):
        - Reduced tire friction on selected wheel

    Behavioral effects (via input manipulation):
        - Constant steering pull toward the affected tire
        - Steering delay (low-pass filtering → “mushy” response)
    """

    fault_name = "tire_pressure_imbalance"

    VALID_WHEELS = {
        "front_left": 0,
        "front_right": 1,
        "rear_left": 2,
        "rear_right": 3,
    }

    def __init__(self,
                 name: str = "tire_pressure_imbalance",
                 severity: float = 1.0,
                 wheel: str = "front_left"):
        """
        Args:
            name: Fault name.
            severity: 0.0 = no fault, 1.0 = maximum effect.
            wheel: Which tire is under-inflated.
        """
        super().__init__(name, severity)

        if wheel not in self.VALID_WHEELS:
            raise ValueError(f"Invalid wheel '{wheel}'. "
                             f"Choose from: {list(self.VALID_WHEELS.keys())}")

        self.wheel_name = wheel
        self.wheel_index = self.VALID_WHEELS[wheel]

        # Physics restoration
        self.original_friction = None

        # Control wrapping
        self.original_apply_control = None
        self.filtered_steering = 0.0  # for low-pass filter steering delay



    def inject(self, target: Any) -> None:
        """Inject both physics and control distortions."""
        if self.is_active:
            self.logger.warning(f"Fault '{self.name}' already active")
            return

        # Check physics support
        if not hasattr(target, "get_physics_control"):
            self.logger.error("Target does not support physics injection")
            return

        # Activate fault
        self.activate()
        self.logger.info(
            f"Injecting hybrid tire imbalance on {self.wheel_name} "
            f"(severity={self.severity})"
        )


        # 1. Apply physics modification
        physics = target.get_physics_control()
        wheel = physics.wheels[self.wheel_index]

        # Save original friction
        self.original_friction = wheel.tire_friction

        # Reduce friction (severity=1 → 50% reduction)
        reduction_factor = 1.0 - 0.5 * self.severity
        wheel.tire_friction *= reduction_factor

        target.apply_physics_control(physics)


        # 2. Wrap apply_control to add steering pull + delay
        if not hasattr(target, "apply_control"):
            self.logger.warning("Target has no apply_control; skipping behavior layer")
            return

        self.original_apply_control = target.apply_control

        # Steering pull direction:
        # under-inflated left → vehicle pulls left
        # else under-inflated tire right -> vehicle pulls right 
        pull_dir = -1.0 if "left" in self.wheel_name else 1.0

        def wrapped_apply_control(control):
            if not self.is_active:
                return self.original_apply_control(control)

            # --- Steering delay (low-pass filter) ---
            # High severity → lower alpha → slower/sloppier steering
            alpha = 0.3 + 0.6 * (1 - self.severity)
            self.filtered_steering = (
                alpha * self.filtered_steering +
                (1 - alpha) * control.steer
            )

            # --- Constant steering pull ---
            max_pull = 0.15  # radians, strong but still drivable
            steering_bias = pull_dir * max_pull * self.severity

            modified_steering = self.filtered_steering + steering_bias

            # Clamp
            control.steer = max(-1.0, min(1.0, modified_steering))

            return self.original_apply_control(control)

        # Install the wrapper
        target.apply_control = wrapped_apply_control



    def remove(self, target: Any) -> None:
        """Restore normal physics and control behavior."""
        if not self.is_active:
            self.logger.warning(f"Fault '{self.name}' is not active")
            return

        self.deactivate()
        self.logger.info(
            f"Removing hybrid tire imbalance fault from {self.wheel_name}"
        )


        # Restore physics
        if self.original_friction is not None:
            physics = target.get_physics_control()
            physics.wheels[self.wheel_index].tire_friction = self.original_friction
            target.apply_physics_control(physics)
            self.original_friction = None


        # Restore control behavior
        if self.original_apply_control is not None:
            target.apply_control = self.original_apply_control
            self.original_apply_control = None

        self.filtered_steering = 0.0
        
        
    def apply_to_image(self, image: np.ndarray) -> np.ndarray:
        return image 