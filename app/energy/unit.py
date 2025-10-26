"""Energy unit definitions and conversion utilities.

This module provides energy-related units, conversion functions, and utilities
for managing power consumption, battery capacity, and energy calculations
within the drone simulation system.

The module includes support for various energy units such as Watt-Hours,
Joules, and provides conversion between different energy measurement systems
commonly used in drone and battery management applications.
"""


class WattHour(float):
    """A unit representing electrical energy in Watt-Hours.

    This class extends float to provide type safety when working with
    energy measurements in the simulation.
    """

    def __new__(cls, value: float) -> "WattHour":
        """Create a new WattHour instance.

        Args:
            value: Energy value in Watt-Hours.

        Returns:
            WattHour: New instance representing the energy value.
        """
        return super().__new__(cls, value)
