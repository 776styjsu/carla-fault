from typing import Any
import numpy as np
from .base_fault import BaseFault


class TractionControlLossFault(BaseFault):
    """
    Simulates loss of traction control (TCS).

    Effects:
    - Excessive wheel slip under acceleration
    - Reduced effective traction at high throttle
    - Unstable yaw during aggressive driving
    """

    fault_name = "traction_control_loss"

    def __init__(self,
                 name: str = "traction_control_loss",
                 severity: float = 1.0):
        """
        Args:
            severity: 0.0 = no fault, 1.0 = complete TCS loss
        """
        super().__init__(name, severity)
        self.original_apply_control = None

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
            f"Injecting traction control loss (severity={self.severity})"
        )

        self.original_apply_control = target.apply_control

        def wrapped_apply_control(control):
            if not self.is_active:
                return self.original_apply_control(control)

            throttle = control.throttle

            # ----------------------------------
            # Traction loss under high throttle
            # ----------------------------------
            # Threshold where slip begins
            slip_threshold = 0.4

            if throttle > slip_threshold:
                excess = throttle - slip_threshold

                # Severity-scaled slip amplification
                slip = excess * self.severity

                # Reduce effective traction (simulate wheel spin)
                throttle = throttle - slip
                throttle = max(0.0, throttle)

            control.throttle = throttle

            return self.original_apply_control(control)

        target.apply_control = wrapped_apply_control


    def remove(self, target: Any) -> None:
        if not self.is_active:
            self.logger.warning(f"Fault '{self.name}' not active")
            return

        self.deactivate()
        self.logger.info("Removing traction control loss fault")

        if self.original_apply_control:
            target.apply_control = self.original_apply_control
            self.original_apply_control = None


    def apply_to_image(self, image: np.ndarray) -> np.ndarray:
        return image