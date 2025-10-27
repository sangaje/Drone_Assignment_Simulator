"""Type definitions and data structures for the drone simulation application.

This module defines the core data types used throughout the drone simulation,
including position coordinates and basic type aliases. It provides type safety
and clear interfaces for the simulation components.

Note:
    For energy-related types and battery management, see app.energy package.
    For geographic positioning, see app.geo package with GeoPoint class.
"""

from typing import NamedTuple, NewType

# Legacy type definition - consider migrating to app.energy.unit.WattHour
WattHour = NewType("WattHour", float)
WattHour.__doc__ = """A unit representing electrical energy in Watt-Hours.

    This is a NewType wrapper around float to provide type safety when
    working with energy measurements in the simulation.

    Note:
        This is a legacy type definition. New code should use
        app.energy.unit.WattHour for full unit system integration.
    """


class Position(NamedTuple):
    """Represents a 2D geographic position in the simulation space.

    Uses longitude and latitude coordinates to specify exact locations
    for drones and other entities in the simulation.

    Note:
        This is a legacy type definition. New code should use
        app.geo.GeoPoint for enhanced geographic functionality with
        WGS84 coordinate system and geodesic calculations.

    Attributes:
        longitude (float): East-west position in decimal degrees.
        latitude (float): North-south position in decimal degrees.
    """

    longitude: float
    latitude: float


# Remove duplicate BatteryStatus class - use app.energy.battery.BatteryStatus instead
# For battery management functionality, import from:
# from app.energy.battery import BatteryStatus
