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

    @current.setter
    def current(self, wh: Energy):
        """Set the current battery charge level.

        Args:
            wh (Energy): New charge level in energy units.

        Raises:
            ValueError: If the new charge level exceeds capacity or is negative.
        """
        if wh < ZERO_ENERGY:
            msg = "Battery charge cannot be negative"
            raise ValueError(msg)
        if wh > self._capacity:
            msg = "Battery charge cannot exceed capacity"
            raise ValueError(msg)
        self._current = wh

    def consume_energy(self, energy: Energy) -> bool:
        """Consume energy from the battery.

        Args:
            energy (Energy): Amount of energy to consume.

        Returns:
            bool: True if energy was successfully consumed, False if insufficient charge.
        """
        if energy < ZERO_ENERGY:
            msg = "Energy consumption cannot be negative"
            raise ValueError(msg)

        if self._current >= energy:
            self._current = type(self._current).from_si(
                float(self._current) - float(energy)
            )
            return True
        return False

    def charge_battery(self, energy: Energy) -> Energy:
        """Add energy to the battery.

        Args:
            energy (Energy): Amount of energy to add to the battery.

        Returns:
            Energy: Amount of energy actually added (may be less than requested if capacity reached)
        """
        if energy < ZERO_ENERGY:
            msg = "Charge energy cannot be negative"
            raise ValueError(msg)

        available_capacity = type(self._capacity).from_si(
            float(self._capacity) - float(self._current)
        )
        actual_charge = type(energy).from_si(
            min(float(energy), float(available_capacity))
        )

        self._current = type(self._current).from_si(
            float(self._current) + float(actual_charge)
        )
        return actual_charge

    def is_empty(self) -> bool:
        """Check if the battery is completely discharged.

        Returns:
            bool: True if battery charge is at or near zero.
        """
        return self._current <= ZERO_ENERGY

    def is_full(self) -> bool:
        """Check if the battery is fully charged.

        Returns:
            bool: True if battery charge equals capacity.
        """
        return self._current >= self._capacity
