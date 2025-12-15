from typing import Any
import time
import numpy as np
from .base_fault import BaseFault


class DegradedBrakePadsFault(BaseFault):
    """
    Simulates worn/degraded brake pads.

    Effects:
    - Reduced braking effectiveness
    - Brake response delay
    - Optional brake fade under sustained braking
    """

    fault_name = "degraded_brake_pads"

    def __init__(self,
                 name: str = "degraded_brake_pads",
                 severity: float = 1.0):
        """
        Args:
            severity: 0.0 = no fault, 1.0 = severely degraded brakes
        """
        super().__init__(name, severity)

        self.original_apply_control = None

        # For brake delay
        self.filtered_brake = 0.0

        # For brake fade
        self.brake_heat = 0.0
        self.last_time = None

    # ------------------------------------------------------------------
    # INJECTION
    # ------------------------------------------------------------------
    def inject(self, target: Any) -> None:
        if self.is_active:
            self.logger.warning(f"Fault '{self.name}' already active")
            return

        if not hasattr(target, "apply_control"):
            self.logger.error("Target does not support apply_control")
            return

        self.activate()
        self.logger.info(
            f"Injecting degraded brake pads (severity={self.severity})"
        )

        self.original_apply_control = target.apply_control

        def wrapped_apply_control(control):
            if not self.is_active:
                return self.original_apply_control(control)

            now = time.time()
            dt = 0.0 if self.last_time is None else (now - self.last_time)
            self.last_time = now

            # -----------------------------
            # 1. Brake delay (low-pass)
            # -----------------------------
            # Higher severity → slower response
            alpha = 0.3 + 0.6 * (1 - self.severity)
            self.filtered_brake = (
                alpha * self.filtered_brake +
                (1 - alpha) * control.brake
            )

            # -----------------------------
            # 2. Brake fade (heat buildup)
            # -----------------------------
            if control.brake > 0.2:
                self.brake_heat += dt * control.brake
            else:
                self.brake_heat -= dt * 0.5  # cooling

            self.brake_heat = max(0.0, min(1.0, self.brake_heat))

            fade_factor = 1.0 - (self.brake_heat * 0.5 * self.severity)

            # -----------------------------
            # 3. Reduced braking power
            # -----------------------------
            max_loss = 0.6  # 60% loss at severity=1
            effectiveness = 1.0 - max_loss * self.severity

            modified_brake = (
                self.filtered_brake *
                effectiveness *
                fade_factor
            )

            control.brake = max(0.0, min(1.0, modified_brake))

            return self.original_apply_control(control)

        target.apply_control = wrapped_apply_control

    # ------------------------------------------------------------------
    # REMOVAL
    # ------------------------------------------------------------------
    def remove(self, target: Any) -> None:
        if not self.is_active:
            self.logger.warning(f"Fault '{self.name}' not active")
            return

        self.deactivate()
        self.logger.info("Removing degraded brake pads fault")

        if self.original_apply_control:
            target.apply_control = self.original_apply_control
            self.original_apply_control = None

        self.filtered_brake = 0.0
        self.brake_heat = 0.0
        self.last_time = None

    # ------------------------------------------------------------------
    # IMAGE INTERFACE (NO-OP)
    # ------------------------------------------------------------------
    def apply_to_image(self, image: np.ndarray) -> np.ndarray:
        return image