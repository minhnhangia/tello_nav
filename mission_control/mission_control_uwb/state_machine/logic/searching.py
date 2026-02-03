#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class SearchingState(BaseState):
    """Handles autonomous exploration with obstacle avoidance."""
    
    def execute(self) -> Optional[MissionState]:
        self.drone.hover()
        return None

