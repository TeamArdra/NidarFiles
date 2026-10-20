"""Which mission a "start" is for -- pure, no rclpy, so it's unit-testable
(test/test_mission_selection.py); the mission nodes (missions/*/mission.py)
are its only users.

radio_command_node publishes /gcs/mission_select {"mission_id": ...} and
then "start" on /gcs/command (via command_node), and every mission node
receives both. They are two topics, so ROS does not guarantee a node sees
the selection before the "start" it belongs to. A selection is therefore
good for ONE start only: take() hands it out and forgets it. A START whose
own selection has not arrived yet finds no selection at all -- every
mission refuses or ignores it (fail closed) -- instead of running whichever
mission an EARLIER START selected.
"""

from __future__ import annotations

from typing import Optional, Tuple

# A START must follow the selection naming its mission within this window
# (radio_command_node publishes both back to back).
SELECTION_MAX_AGE_S = 5.0


class MissionSelection:
    def __init__(self, max_age_s: float = SELECTION_MAX_AGE_S) -> None:
        self._max_age_s = max_age_s
        self._selected: Optional[Tuple[Optional[str], float]] = None  # (mission id, received at)

    def update(self, mission_id: Optional[str], now: float) -> None:
        """A /gcs/mission_select arrived (mission_id None if it had none)."""
        self._selected = (mission_id, now)

    def take(self, now: float) -> Tuple[Optional[str], bool]:
        """For a "start": (selected mission id, whether the selection is
        fresh), then forget it. (None, False) = no selection since the
        last start."""
        selected, self._selected = self._selected, None
        if selected is None:
            return None, False
        mission_id, at = selected
        return mission_id, now - at <= self._max_age_s

    def take_for(self, mission_id: str, now: float) -> Tuple[bool, Optional[str]]:
        """For a "start" arriving at `mission_id`'s node (uses up the
        selection, like take()): (for_us, problem). for_us False = a fresh
        selection names another mission -- not ours, not a failure.
        Otherwise problem is None when this START may run, else why not."""
        selected, fresh = self.take(now)
        if fresh and selected != mission_id:
            return False, None
        if selected is None:
            return True, "no mission selection received"
        if selected != mission_id:
            return True, f"selected mission is {selected!r}, not {mission_id!r}"
        if not fresh:
            return True, "mission selection is stale"
        return True, None
