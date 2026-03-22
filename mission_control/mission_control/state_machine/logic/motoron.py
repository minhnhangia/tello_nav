#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class MotorOnState(BaseState):
    """Handles motor on action."""

    def execute(self) -> Optional[MissionState]:
        """Execute MOTOR_ON state logic."""        
        self.drone.execute_action(
            "motoron",
            MissionState.IDLE,
            MissionState.IDLE,
            max_retries=2
        )
        return None