"""Universal unit system for type-safe physical quantity management.

This package provides a comprehensive, type-safe unit system designed specifically
for drone simulation applications. It implements automatic SI unit conversion,
unit family type checking, and mathematical operations between compatible units
while preventing mixing of incompatible physical quantities.

Architecture:
    The unit system is organized into specialized modules:

    - unit_base: Foundation Unit class with family management system
    - unit_float: Float-based units with automatic SI conversion
    - unit_angle: Angular units (Radian, Degree) for orientation
    - unit_distance: Distance units (Meter, Kilometer) for positioning
    - unit_time: Time units (Second, Minute, Hour) for duration

Key Features:
    - Type Safety: Prevents mixing incompatible units (e.g., meters + seconds)
    - SI Conversion: Automatic conversion to SI units for internal storage
    - Unit Operations: Full arithmetic support between compatible units
    - Human Readable: Natural string representations with proper symbols
    - Extensible: Easy to add new unit types and families

Unit Families:
    Each unit family represents a distinct physical quantity:

    - Angle Family: Radian (root), Degree
    - Distance Family: Meter (root), Kilometer
    - Time Family: Second (root), Minute, Hour

Common Use Cases:
    - Drone positioning and navigation (distances, coordinates)
    - Flight planning and timing (durations, speeds)
    - Orientation and control (angles, rotations)
    - Sensor measurements and ranges
    - Mission parameters and constraints

Example:
    >>> from dronesim.unit import Meter, Kilometer, Degree, Second
    >>>
    >>> # Create units with automatic SI conversion
    >>> altitude = Meter(150)  # 150 meters AGL
    >>> range_limit = Kilometer(5.2)  # 5.2 km max range
    >>> heading = Degree(45)  # 45° northeast
    >>> flight_time = Second(300)  # 5 minutes
    >>>
    >>> # Type-safe operations within same family
    >>> total_distance = altitude + range_limit.to(Meter)  # ✅ OK: both distances
    >>> print(total_distance)  # "5350.0 m"
    >>>
    >>> # Cross-family operations are prevented
    >>> # result = heading + altitude  # ❌ ERROR: incompatible units
    >>>
    >>> # Unit conversions within families
    >>> heading_rad = heading.to(Radian)  # Convert to radians
    >>> range_m = range_limit.to(Meter)  # Convert to meters

Available Units:
    Angular Units:
        - Radian: SI base unit for angles
        - Degree: Common angular unit (360° = full rotation)

    Distance Units:
        - Meter: SI base unit for length
        - Kilometer: 1000 meters

    Time Units:
        - Second: SI base unit for time
        - Minute: 60 seconds
        - Hour: 3600 seconds

    Velocity Units:
        - MeterPerSecond: SI base unit for velocity (m/s)
        - MetersPerHour: Very slow velocities (m/h)
        - KilometersPerSecond: High-speed applications (km/s)
        - KilometersPerHour: Common aviation unit (km/h)
"""

# Import all unit definitions
from .unit_angle import Angle, Degree, Radian
from .unit_base import Unit
from .unit_distance import Kilometer, Length, Meter
from .unit_float import UnitFloat
from .unit_power import Kilowatt, Power, Watt
from .unit_time import ClockTime, Hour, Minute, Second, Time
from .unit_velocity import (
    KilometersPerHour,
    KilometersPerSecond,
    MeterPerSecond,
    MetersPerHour,
    Velocity,
)

# Define public API
__all__ = [
    # Base classes
    "Unit",
    "UnitFloat",
    # Angular units
    "Radian",
    "Degree",
    "Angle",
    # Distance units
    "Meter",
    "Kilometer",
    "Length",
    # Time units
    "Second",
    "Minute",
    "Hour",
    "ClockTime",
    "Time",
    # Velocity units
    "MeterPerSecond",
    "MetersPerHour",
    "KilometersPerSecond",
    "KilometersPerHour",
    "Velocity",
    # Power units
    "Watt",
    "Kilowatt",
    "Power",
]
