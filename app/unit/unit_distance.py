"""Distance and length unit definitions for spatial measurements.

This module provides distance units essential for drone simulation and
navigation. All distance measurements are internally stored in meters
(the SI base unit) for consistency, while supporting input and display
in various distance scales.

These units are commonly used for:
- Drone flight distances and ranges
- Altitude and elevation measurements
- GPS coordinate precision and accuracy
- Sensor detection ranges
- Mission planning and waypoint distances

Classes:
    Meter: Base distance unit in meters (SI unit).
    Kilometer: Distance unit in kilometers with automatic meter conversion.

Type Aliases:
    Length: Union type for all distance units (Meter | Kilometer).

Example:
    >>> flight_range = Kilometer(25.5)  # 25.5 km range
    >>> print(flight_range)  # "25.5 km"
    >>> print(float(flight_range))  # 25500.0 (meters in SI)
    >>>
    >>> altitude = Meter(120)  # 120 meters AGL
    >>> print(altitude.to(Kilometer))  # 0.12 (km)
"""

from __future__ import annotations

from .unit_float import UnitFloat


class Meter(UnitFloat):
    """Distance unit: Meter (SI base unit for length).

    The Meter class represents distance measurements in meters, which is
    the SI base unit for length. This unit provides the foundation for
    all distance measurements in the simulation system.

    Commonly used for:
    - Precise positioning and navigation
    - Altitude measurements (AGL/MSL)
    - Sensor ranges and detection distances
    - Short to medium range measurements
    - GPS coordinate precision

    Attributes:
        IS_FAMILY_ROOT (bool): True, indicating this is the root distance unit.
        SCALE_TO_SI (float): 1.0, no conversion needed for SI base unit.
        SYMBOL (str): "m", the standard symbol for meters.

    Example:
        >>> altitude = Meter(150.5)  # 150.5 meters
        >>> print(altitude)  # "150.5 m"
        >>> print(float(altitude))  # 150.5
    """

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "m"


class Kilometer(Meter):
    """Distance unit: Kilometer (1000 meters).

    The Kilometer class represents distance measurements in kilometers.
    Each kilometer equals 1000 meters, and values are automatically
    converted to meters for internal storage and calculations.

    Commonly used for:
    - Long-range flight planning
    - Mission distances and coverage areas
    - Geographic scale measurements
    - Flight range specifications
    - Regional mapping and surveying

    Attributes:
        SCALE_TO_SI (float): 1000.0, conversion factor from km to meters.
        SYMBOL (str): "km", the standard symbol for kilometers.

    Example:
        >>> mission_range = Kilometer(50.2)  # 50.2 km range
        >>> print(mission_range)  # "50.2 km"
        >>> print(float(mission_range))  # 50200.0 (meters)
    """

    SCALE_TO_SI = 1000.0
    SYMBOL = "km"


Length = Meter | Kilometer  # Type alias for any length unit
