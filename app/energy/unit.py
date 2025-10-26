"""Energy unit definitions and conversion utilities.

This module provides energy-related units, conversion functions, and utilities
for managing power consumption, battery capacity, and energy calculations
within the drone simulation system.

The module includes support for various energy units such as Watt-Hours,
Joules, and provides conversion between different energy measurement systems
commonly used in drone and battery management applications.
"""
from app.unit import UnitFloat


class WattSecond(UnitFloat):
    """A unit representing energy in Watt-Seconds (Joules).

    This class extends float to provide type safety when working with
    energy measurements in the simulation.
    """

    IS_FAMILY_ROOT: bool = True
    SCALE_TO_SI: float = 1.0  # 1 Watt-Second = 1 Joule
    SYMBOL: str = "Ws"

class WattMinute(WattSecond):
    """A unit representing energy in Watt-Minutes.

    This class extends float to provide type safety when working with
    energy measurements in the simulation.
    """

    SCALE_TO_SI: float = 60.0  # 1 Watt-Minute = 60 Joules
    SYMBOL: str = "Wm"

class WattHour(WattSecond):
    """A unit representing electrical energy in Watt-Hours.

    This class extends float to provide type safety when working with
    energy measurements in the simulation.
    """

    SCALE_TO_SI: float = 3600.0  # 1 Watt-Hour = 3600 Joules
    SYMBOL: str = "Wh"

class KilowattHour(WattSecond):
    """A unit representing electrical energy in Kilowatt-Hours.

    This class extends float to provide type safety when working with
    energy measurements in the simulation.
    """

    SCALE_TO_SI: float = 3_600_000.0  # 1 Kilowatt-Hour = 3,600,000 Joules
    SYMBOL: str = "kWh"

Energy = WattSecond | WattMinute | WattHour | KilowattHour
ZERO_ENERGY = WattSecond(0.0)
