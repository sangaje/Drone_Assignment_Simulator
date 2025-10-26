"""Type definitions for the drone simulation application."""
from typing import NamedTuple


class Position(NamedTuple):
    """Represents a 2D position in the simulation space."""

    longitude: float
    latitude: float


class BatteryStatus:
    """Represents the battery status of a vehicle."""

    _capacity: float
    _current: float

    def __init__(self, capacity: float, current: float):
        """Initialize BatteryStatus with capacity and current charge in Watt-Hours."""
        self._capacity = capacity
        self._current = current

    def __iadd__(self, wh: float):
        """Increase current charge by given Watt-Hours."""
        self._current = min(self._capacity, self._current + wh)
        return self

    def __isub__(self, wh: float):
        """Decrease current charge by given Watt-Hours."""
        self._current = max(0.0, self._current - wh)
        return self

    @property
    def percentage(self) -> float:
        """Get current battery percentage."""
        return (self._current / self._capacity) * 100.0

    @property
    def capacity(self) -> float:
        """Get battery capacity in Watt-Hours."""
        return self._capacity

    @property
    def current(self) -> float:
        """Get current charge in Watt-Hours."""
        return self._current
