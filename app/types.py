"""Type definitions and data structures for the drone simulation application.

This module defines the core data types used throughout the drone simulation,
including position coordinates, battery status management, and energy units.
These types provide type safety and clear interfaces for the simulation components.
"""

from typing import NamedTuple, NewType

WattHour = NewType("WattHour", float)
WattHour.__doc__ = """A unit representing electrical energy in Watt-Hours.

    This is a NewType wrapper around float to provide type safety when
    working with energy measurements in the simulation.
    """


class Position(NamedTuple):
    """Represents a 2D geographic position in the simulation space.

    Uses longitude and latitude coordinates to specify exact locations
    for drones and other entities in the simulation.

    Attributes:
        longitude (float): East-west position in decimal degrees.
        latitude (float): North-south position in decimal degrees.
    """

    longitude: float
    latitude: float


class BatteryStatus:
    """Manages the battery status and energy consumption of a vehicle.

    This class tracks both the maximum capacity and current charge level
    of a battery, providing methods for charging, discharging, and
    monitoring battery health and percentage.

    Attributes:
        _capacity (WattHour): Maximum battery capacity in Watt-Hours.
        _current (WattHour): Current charge level in Watt-Hours.
    """

    _capacity: WattHour
    _current: WattHour

    def __init__(self, capacity: WattHour, current: WattHour):
        """Initialize BatteryStatus with specified capacity and current charge.

        Args:
            capacity (WattHour): Maximum battery capacity in Watt-Hours.
            current (WattHour): Initial charge level in Watt-Hours.

        Raises:
            ValueError: If current charge exceeds capacity or either value is negative.
        """
        if capacity < 0 or current < 0:
            raise ValueError
        if current > capacity:
            raise ValueError
        self._capacity = capacity
        self._current = current

    def __iadd__(self, wh: WattHour):
        """Charge the battery by adding the specified amount of energy.

        Args:
            wh (WattHour): Amount of energy to add to the battery.

        Returns:
            BatteryStatus: Self for method chaining.

        Note:
            The charge will not exceed the battery's maximum capacity.
        """
        self._current = min(self._capacity, self._current + wh)
        return self

    def __isub__(self, wh: WattHour):
        """Discharge the battery by removing the specified amount of energy.

        Args:
            wh (WattHour): Amount of energy to remove from the battery.

        Returns:
            BatteryStatus: Self for method chaining.

        Note:
            The charge will not go below zero.
        """
        self._current = max(0.0, self._current - wh)
        return self

    @property
    def percentage(self) -> float:
        """Get the current battery charge as a percentage of capacity.

        Returns:
            float: Battery percentage from 0.0 to 100.0.
        """
        return (self._current / self._capacity) * 100.0

    @property
    def capacity(self) -> WattHour:
        """Get the maximum battery capacity.

        Returns:
            WattHour: Maximum battery capacity in Watt-Hours.
        """
        return self._capacity

    @property
    def current(self) -> WattHour:
        """Get the current battery charge level.

        Returns:
            WattHour: Current charge level in Watt-Hours.
        """
        return self._current
