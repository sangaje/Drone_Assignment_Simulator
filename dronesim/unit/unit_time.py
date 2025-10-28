"""Time unit definitions for the drone simulation.

This module provides time-based units including seconds, minutes, and hours.
All time units are based on the Second class as the SI base unit, with automatic
conversion between different time scales.

Classes:
    Second: Base time unit in seconds (SI unit).
    Minute: Time unit representing 60 seconds.
    Hour: Time unit representing 3600 seconds (1 hour).

Type Aliases:
    Time: Union type for all time units (Second | Minute | Hour).

Example:
    >>> duration = Hour(2.5)  # 2.5 hours
    >>> print(duration)  # "2.5 h"
    >>> print(float(duration))  # 9000.0 (seconds in SI)
    >>>
    >>> meeting_time = Minute(30)  # 30 minutes
    >>> total_seconds = float(meeting_time)  # 1800.0 seconds
"""

from __future__ import annotations

from .unit_float import UnitFloat


class Second(UnitFloat):
    """Time unit: Second (SI base unit for time).

    The Second class represents time durations in seconds, which is the base
    SI unit for time. All other time units (Minute, Hour) are derived from
    this class and automatically convert to seconds internally.

    Attributes:
        IS_FAMILY_ROOT (bool): True, indicating this is the root time unit.
        SCALE_TO_SI (float): 1.0, no conversion needed for SI base unit.
        SYMBOL (str): "s", the standard symbol for seconds.

    Example:
        >>> time_interval = Second(5.5)
        >>> print(time_interval)  # "5.5 s"
        >>> print(float(time_interval))  # 5.5
    """

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "s"


class Minute(Second):
    """Time unit: Minute (60 seconds).

    The Minute class represents time durations in minutes. Each minute
    equals 60 seconds, and values are automatically converted to seconds
    for internal storage and calculations.

    Attributes:
        SCALE_TO_SI (float): 60.0, conversion factor from minutes to seconds.
        SYMBOL (str): "min", the standard symbol for minutes.

    Example:
        >>> duration = Minute(2.5)  # 2.5 minutes
        >>> print(duration)  # "2.5 min"
        >>> print(float(duration))  # 150.0 (seconds)
    """

    SCALE_TO_SI = 60.0
    SYMBOL = "min"


class Hour(Second):
    """Time unit: Hour (3600 seconds).

    The Hour class represents time durations in hours. Each hour equals
    3600 seconds (60 minutes × 60 seconds), and values are automatically
    converted to seconds for internal storage and calculations.

    Attributes:
        SCALE_TO_SI (float): 3600.0, conversion factor from hours to seconds.
        SYMBOL (str): "h", the standard symbol for hours.

    Example:
        >>> flight_time = Hour(1.25)  # 1.25 hours (1 hour 15 minutes)
        >>> print(flight_time)  # "1.25 h"
        >>> print(float(flight_time))  # 4500.0 (seconds)
    """

    SCALE_TO_SI = 3600.0
    SYMBOL = "h"


Time = Second | Minute | Hour  # Type alias for any time unit
