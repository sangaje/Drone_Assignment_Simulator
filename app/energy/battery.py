"""Battery management system for drone energy simulation.

This module provides comprehensive battery management functionality including
battery models, charging/discharging algorithms, capacity degradation models,
and energy consumption calculations for drone operations.

The module supports various battery types, implements realistic battery
behavior models, and provides interfaces for monitoring battery health,
state of charge, and energy optimization strategies within the simulation.

Key features:
- Battery state management and monitoring
- Charging and discharging simulation
- Capacity degradation over time
- Energy consumption prediction
- Battery health diagnostics
- Temperature effects on battery performance
"""
from .unit import ZERO_ENERGY, Energy


class BatteryStatus:
    """Manages the battery status and energy consumption of a vehicle.

    This class tracks both the maximum capacity and current charge level
    of a battery, providing methods for charging, discharging, and
    monitoring battery health and percentage.

    Attributes:
        _capacity (WattHour): Maximum battery capacity in Watt-Hours.
        _current (WattHour): Current charge level in Watt-Hours.
    """

    _capacity: Energy
    _current: Energy

    def __init__(self, capacity: Energy, current: Energy):
        """Initialize BatteryStatus with specified capacity and current charge.

        Args:
            capacity (WattHour): Maximum battery capacity in Watt-Hours.
            current (WattHour): Initial charge level in Watt-Hours.

        Raises:
            ValueError: If current charge exceeds capacity or either value is negative.
        """
        if capacity < ZERO_ENERGY or current < ZERO_ENERGY:
            raise ValueError
        if current > capacity:
            raise ValueError

        self._capacity = capacity
        self._current = current

    @property
    def percentage(self) -> float:
        """Get the current battery charge as a percentage of capacity.

        Returns:
            float: Battery percentage from 0.0 to 100.0.
        """
        return (float(self._current) / float(self._capacity)) * 100.0

    @property
    def capacity(self) -> Energy:
        """Get the maximum battery capacity.

        Returns:
            WattHour: Maximum battery capacity in Watt-Hours.
        """
        return self._capacity

    @property
    def current(self) -> Energy:
        """Get the current battery charge level.

        Returns:
            WattHour: Current charge level in Watt-Hours.
        """
        return self._current
