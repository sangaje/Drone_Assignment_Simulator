"""Velocity unit definitions for drone simulation.

This module provides velocity and speed units for drone movement calculations.
All velocity units are based on MeterPerSecond as the SI base unit (m/s), with
automatic conversion between different velocity scales commonly used in aviation
and ground vehicle applications.

The velocity units support both metric and aviation-standard measurements,
enabling accurate speed calculations for drone flight dynamics, navigation,
and mission planning scenarios.

Classes:
    MeterPerSecond: SI base velocity unit (m/s).
    MetersPerHour: Slow-speed applications (m/h).
    KilometersPerSecond: High-speed applications (km/s).
    KilometersPerHour: Common automotive/aviation unit (km/h).

Type Aliases:
    Velocity: Union type for all velocity units.

Example:
    >>> cruise_speed = KilometersPerHour(50)  # 50 km/h cruise speed
    >>> print(cruise_speed)  # "50.0 km/h"
    >>> print(float(cruise_speed))  # 13.888... (m/s in SI units)
    >>>
    >>> max_speed = MeterPerSecond(25)  # 25 m/s maximum speed
    >>> print(max_speed.to(KilometersPerHour))  # "90.0 km/h"
"""

from __future__ import annotations

from .unit_float import UnitFloat


class MeterPerSecond(UnitFloat):
    """Velocity unit: Meters per Second (SI base unit for velocity).

    The MeterPerSecond class represents velocity in meters per second (m/s),
    which is the standard SI unit for velocity. All other velocity units in
    this module derive from this class and automatically convert to m/s
    for internal calculations.

    This unit is commonly used in physics calculations, engineering
    applications, and scientific measurements where precise velocity
    values are required.

    Attributes:
        IS_FAMILY_ROOT (bool): True, indicating this is the root velocity unit.
        SCALE_TO_SI (float): 1.0, no conversion needed for SI base unit.
        SYMBOL (str): "m/s", the standard symbol for meters per second.

    Example:
        >>> drone_speed = MeterPerSecond(15.5)  # 15.5 m/s
        >>> print(drone_speed)  # "15.5 m/s"
        >>> print(float(drone_speed))  # 15.5 (SI value)
    """

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "m/s"


class MetersPerHour(MeterPerSecond):
    """Velocity unit: Meters per Hour.

    The MetersPerHour class represents very slow velocities in meters per hour,
    useful for applications requiring precise measurement of slow-moving objects
    or processes. This unit converts to the SI base unit (m/s) by dividing by 3600.

    This unit is rarely used in drone applications but may be useful for
    ground-based equipment, slow autonomous vehicles, or precision positioning
    systems that operate at very low speeds.

    Attributes:
        SCALE_TO_SI (float): 1/3600, converts hours to seconds.
        SYMBOL (str): "m/h", the symbol for meters per hour.

    Example:
        >>> slow_speed = MetersPerHour(360)  # 360 m/h
        >>> print(slow_speed)  # "360.0 m/h"
        >>> print(float(slow_speed))  # 0.1 (m/s in SI units)
    """

    SCALE_TO_SI = 1.0 / 3600.0
    SYMBOL = "m/h"

class KilometersPerSecond(MeterPerSecond):
    """Velocity unit: Kilometers per Second.

    The KilometersPerSecond class represents extremely high velocities in
    kilometers per second, typically used for orbital mechanics, spacecraft,
    or theoretical high-speed calculations. This unit converts to SI base
    unit (m/s) by multiplying by 1000.

    While not commonly used in typical drone operations, this unit may be
    relevant for space-based drones, satellite operations, or advanced
    propulsion system calculations.

    Attributes:
        SCALE_TO_SI (float): 1000.0, converts kilometers to meters.
        SYMBOL (str): "km/s", the symbol for kilometers per second.

    Example:
        >>> orbital_speed = KilometersPerSecond(7.8)  # 7.8 km/s orbital velocity
        >>> print(orbital_speed)  # "7.8 km/s"
        >>> print(float(orbital_speed))  # 7800.0 (m/s in SI units)
    """

    SCALE_TO_SI = 1000.0
    SYMBOL = "km/s"

class KilometersPerHour(MeterPerSecond):
    """Velocity unit: Kilometers per Hour.

    The KilometersPerHour class represents velocity in kilometers per hour (km/h),
    which is one of the most commonly used velocity units in aviation, automotive,
    and general transportation applications. This unit converts to SI base unit
    (m/s) by multiplying by 1000 and dividing by 3600.

    This unit is ideal for drone specifications, flight planning, and human-readable
    speed references, as it provides intuitive values for typical drone operating
    speeds (10-100 km/h range for most consumer and commercial drones).

    Attributes:
        SCALE_TO_SI (float): 1000/3600 ≈ 0.2778, converts km/h to m/s.
        SYMBOL (str): "km/h", the symbol for kilometers per hour.

    Example:
        >>> cruise_speed = KilometersPerHour(72)  # 72 km/h
        >>> print(cruise_speed)  # "72.0 km/h"
        >>> print(float(cruise_speed))  # 20.0 (m/s in SI units)
    """

    SCALE_TO_SI = 1000.0 / 3600.0  # ≈ 0.2778
    SYMBOL = "km/h"

# Type alias for all velocity unit types
Velocity = (
    MeterPerSecond | MetersPerHour | KilometersPerSecond | KilometersPerHour
)
