#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class CompletingMissionState(BaseState):
    """Handles post-landing marker status update."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute COMPLETING_MISSION state logic."""
        if not self.context.is_blind_landing:
            self.node.get_logger().info(
                "COMPLETING_MISSION: Marking marker as landed and completing mission."
            )
            self.marker_handler.mark_current_marker_landed()
        else:
            self.node.get_logger().warning(
                "COMPLETING_MISSION: Blind landing. NOT marking marker as landed."
            )
        return MissionState.RESETTING