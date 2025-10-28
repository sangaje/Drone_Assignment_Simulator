"""Power unit definitions for drone simulation energy calculations.

This module provides power units for drone energy consumption modeling,
motor power specifications, and electrical system calculations. All power
units are based on Watt as the SI base unit with automatic conversion
between different power scales commonly used in drone applications.

The power units support calculations for various drone electrical systems
including motors, batteries, sensors, and control systems. They integrate
seamlessly with the energy unit system to enable comprehensive power
consumption modeling and battery life calculations.

Classes:
    Watt: SI base power unit (W).
    Kilowatt: High-power applications (kW).

Type Aliases:
    Power: Union type for all power units.

Example:
    >>> motor_power = Watt(150)  # 150W motor power
    >>> print(motor_power)  # "150.0 W"
    >>> print(float(motor_power))  # 150.0 (W in SI units)
    >>>
    >>> high_power = Kilowatt(2.5)  # 2.5kW industrial motor
    >>> print(high_power.to(Watt))  # 2500.0 W
    >>>
    >>> # Power calculations for flight time estimation
    >>> battery_capacity = 55.5  # Wh
    >>> flight_time_hours = battery_capacity / float(motor_power)  # Hours
"""

from __future__ import annotations

from .unit_float import UnitFloat


class Watt(UnitFloat):
    """Power unit: Watt (SI base unit for power).

    The Watt class represents power in Watts, which is the base SI unit
    for power. All other power units (Kilowatt, Megawatt) are derived
    from this class and automatically convert to Watts internally.

    Attributes:
        IS_FAMILY_ROOT (bool): True, indicating this is the root power unit.
        SCALE_TO_SI (float): 1.0, no conversion needed for SI base unit.
        SYMBOL (str): "W", the standard symbol for Watts.

    Example:
        >>> power_output = Watt(1500.0)
        >>> print(power_output)  # "1500.0 W"
        >>> print(float(power_output))  # 1500.0
    """

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "W"


class Kilowatt(Watt):
    """Power unit: Kilowatt (1000 Watts).

    The Kilowatt class represents power in Kilowatts. Each Kilowatt
    equals 1000 Watts, and values are automatically converted to Watts
    for internal storage and calculations.

    Attributes:
        SCALE_TO_SI (float): 1000.0, conversion factor from Kilowatts to Watts.
        SYMBOL (str): "kW", the standard symbol for Kilowatts.

    Example:
        >>> motor_power = Kilowatt(2.5)  # 2.5 kW
        >>> print(motor_power)  # "2.5 kW"
        >>> print(float(motor_power))  # 2500.0 (Watts)
    """

    SCALE_TO_SI = 1000.0
    SYMBOL = "kW"


Power = Watt | Kilowatt
