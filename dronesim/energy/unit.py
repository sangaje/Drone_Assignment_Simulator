"""Energy unit definitions and conversion utilities.

This module provides energy-related units, conversion functions, and utilities for managing power
consumption, battery capacity, and energy calculations within the drone simulation system.

The module includes support for various energy units such as Watt-Hours, Joules, and provides
conversion between different energy measurement systems commonly used in drone and battery
management applications.
"""

from dronesim.unit import UnitFloat


class WattSecond(UnitFloat):
    """A unit representing energy in Watt-Seconds (Joules).

    This is the SI base unit for energy in the simulation. Watt-Seconds
    are equivalent to Joules and provide the foundation for all energy
    calculations in the drone battery system.

    Example:
        >>> energy = WattSecond(3600)  # 3600 Joules
        >>> float(energy)  # SI value in Joules
        3600.0
    """

    IS_FAMILY_ROOT: bool = True
    SCALE_TO_SI: float = 1.0  # 1 Watt-Second = 1 Joule
    SYMBOL: str = "Ws"


class WattMinute(WattSecond):
    """A unit representing energy in Watt-Minutes.

    Useful for medium-duration energy measurements. One Watt-Minute
    equals 60 Joules and is commonly used in battery capacity calculations
    for shorter time periods.

    Example:
        >>> energy = WattMinute(1)  # 1 Watt-Minute
        >>> float(energy)  # 60 Joules in SI units
        60.0
    """

    SCALE_TO_SI: float = 60.0  # 1 Watt-Minute = 60 Joules
    SYMBOL: str = "Wm"


class WattHour(WattSecond):
    """A unit representing electrical energy in Watt-Hours.

    The most common unit for battery capacity and energy storage in
    consumer electronics and drone applications. One Watt-Hour equals
    3600 Joules.

    Example:
        >>> battery_capacity = WattHour(100)  # 100 Wh battery
        >>> float(battery_capacity)  # 360000 Joules in SI units
        360000.0
    """

    SCALE_TO_SI: float = 3600.0  # 1 Watt-Hour = 3600 Joules
    SYMBOL: str = "Wh"


class KilowattHour(WattSecond):
    """A unit representing electrical energy in Kilowatt-Hours.

    Used for large energy storage systems and grid-scale applications.
    One Kilowatt-Hour equals 3.6 million Joules and is typically used
    for high-capacity industrial drone systems.

    Example:
        >>> large_battery = KilowattHour(10)  # 10 kWh battery
        >>> float(large_battery)  # 36,000,000 Joules in SI units
        36000000.0
    """

    SCALE_TO_SI: float = 3_600_000.0  # 1 Kilowatt-Hour = 3,600,000 Joules
    SYMBOL: str = "kWh"


# Type alias for all energy unit types
Energy = WattSecond | WattMinute | WattHour | KilowattHour

# Constant for zero energy value, useful for comparisons and calculations
ZERO_ENERGY = WattSecond(0.0)
