"""Geographic coordinate utilities for drone simulation.

This package provides type-safe geographic coordinate classes and geodesic
calculations for drone navigation and positioning. Built on the WGS84 ellipsoid
model for accurate Earth surface distance and bearing computations.

Components:
    GeoPoint: Geographic point with latitude/longitude coordinates
    Latitude: Type-safe latitude coordinate in degrees (-90° to +90°)
    Longitude: Type-safe longitude coordinate in degrees (-180° to +180°)

Key Features:
    • WGS84 ellipsoid model for accurate distance calculations
    • Type-safe coordinates preventing lat/lon mixing errors
    • Efficient in-place coordinate updates for real-time simulation
    • Geodesic forward/inverse calculations for navigation
    • Waypoint passage detection for discrete time stepping

Typical Usage:
    >>> from dronesim.geo import GeoPoint, Latitude, Longitude
    >>> from dronesim.unit import Degree, Meter
    >>>
    >>> # Create mission waypoints
    >>> home_base = GeoPoint.from_deg(37.5665, 126.9780)
    >>> target_zone = GeoPoint.from_deg(37.5700, 126.9850)
    >>>
    >>> # Calculate flight parameters
    >>> mission_distance = home_base.distance_to(target_zone)
    >>> print(f"Distance: {float(mission_distance):.0f}m")
    >>>
    >>> # Navigate using bearing and distance
    >>> bearing = Degree(45)  # Northeast heading
    >>> checkpoint = home_base.forward(bearing, Meter(500))
"""

from .geo_point import GeoPoint, Latitude, Longitude

__all__ = ["GeoPoint", "Latitude", "Longitude"]
