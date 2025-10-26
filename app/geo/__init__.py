"""Geographic coordinate utilities and functions.

This module provides utilities for working with geographic coordinates,
distance calculations, and spatial operations for the drone simulation.

The module defines coordinate types (Latitude, Longitude) that extend the unit
system, and a GeoPoint dataclass for representing geographic locations with
geodesic distance calculations using the WGS84 ellipsoid.
"""

from dataclasses import dataclass

from app.unit import Angle, Degree, Length, Meter, Radian
from pyproj import Geod

# WGS84 geodesic calculator for accurate Earth surface calculations
_WGS84 = Geod(ellps="WGS84")


class Latitude(Degree):
    """A unit representing latitude in degrees.

    This class extends Degree to provide type safety when working with
    latitude measurements in the simulation. Latitude values are typically
    in the range of -90 to +90 degrees, representing positions from the
    South Pole to the North Pole.

    The class inherits SCALE_TO_SI from Degree to ensure proper conversion
    to radians for internal calculations while displaying in degrees.

    Example:
        >>> lat = Latitude(37.5665)  # Seoul latitude
        >>> float(lat)  # Internal radian value
        0.6556588...
        >>> str(lat)    # Display value
        '37.5665 °N/S'
    """

    IS_FAMILY_ROOT = True
    SYMBOL = "°N/S"


class Longitude(Degree):
    """A unit representing longitude in degrees.

    This class extends Degree to provide type safety when working with
    longitude measurements in the simulation. Longitude values are typically
    in the range of -180 to +180 degrees, representing positions from the
    International Date Line westward to eastward.

    The class inherits SCALE_TO_SI from Degree to ensure proper conversion
    to radians for internal calculations while displaying in degrees.

    Example:
        >>> lon = Longitude(126.9780)  # Seoul longitude
        >>> float(lon)  # Internal radian value
        2.2161834...
        >>> str(lon)    # Display value
        '126.978 °E/W'
    """

    IS_FAMILY_ROOT = True
    SYMBOL = "°E/W"


@dataclass
class GeoPoint:
    """Represents a geographic point with latitude and longitude coordinates.

    This class encapsulates a geographic location on Earth's surface using
    WGS84 coordinates, providing methods for geodesic distance calculations
    and spatial operations. The class is mutable (unlike NamedTuple) to allow
    efficient coordinate updates in performance-critical simulations.

    All distance calculations use the WGS84 ellipsoid model for accuracy,
    accounting for Earth's oblate shape rather than assuming a perfect sphere.

    Attributes:
        latitude (Latitude): Latitude coordinate in degrees (North/South position).
        longitude (Longitude): Longitude coordinate in degrees (East/West position).

    Example:
        >>> # Create points for Seoul and Busan
        >>> seoul = GeoPoint.from_deg(37.5665, 126.9780)
        >>> busan = GeoPoint.from_deg(35.1796, 129.0756)
        >>>
        >>> # Calculate distance
        >>> distance = seoul.distance_to(busan)
        >>> print(f"Distance: {float(distance)/1000:.1f} km")
        Distance: 325.4 km
        >>>
        >>> # Move to a new location
        >>> new_point = seoul.forward(Degree(90), Meter(1000))  # 1km east
        >>> seoul.move_to(Degree(0), Meter(500))  # Move 500m north in-place
    """

    latitude: Latitude
    longitude: Longitude

    @classmethod
    def from_deg(cls, lat: float, lon: float) -> "GeoPoint":
        """Create a GeoPoint from latitude and longitude values in degrees.

        Convenience constructor for creating GeoPoint instances from decimal
        degree coordinates, which is the most common format for geographic data.

        Args:
            lat (float): Latitude in decimal degrees (-90 to +90).
                        Negative values represent South, positive North.
            lon (float): Longitude in decimal degrees (-180 to +180).
                        Negative values represent West, positive East.

        Returns:
            GeoPoint: New geographic point with the specified coordinates.

        Example:
            >>> # Seoul coordinates
            >>> seoul = GeoPoint.from_deg(37.5665, 126.9780)
            >>> # Antarctic research station (negative latitude)
            >>> mcmurdo = GeoPoint.from_deg(-77.8419, 166.6863)
        """
        return cls(Latitude(lat), Longitude(lon))

    @classmethod
    def from_rad(cls, lat: float, lon: float) -> "GeoPoint":
        """Create a GeoPoint from latitude and longitude values in radians.

        Alternative constructor for creating GeoPoint instances from radian
        coordinates, useful when working with mathematical calculations or
        data sources that provide coordinates in radians.

        Args:
            lat (float): Latitude in radians (-π/2 to +π/2).
                        Negative values represent South, positive North.
            lon (float): Longitude in radians (-π to +π).
                        Negative values represent West, positive East.

        Returns:
            GeoPoint: New geographic point with coordinates converted from radians.

        Example:
            >>> import math
            >>> # Create point using radian values
            >>> point = GeoPoint.from_rad(math.pi/4, math.pi/3)  # 45°N, 60°E
            >>> print(point)
            GeoPoint(latitude=45.0 °N/S, longitude=60.0 °E/W)
        """
        return cls(Radian(lat).to(Latitude), Radian(lon).to(Longitude))

    def distance_to(self, other: "GeoPoint") -> Meter:
        """Calculate the geodesic distance to another GeoPoint.

        Uses the WGS84 ellipsoid model to compute the shortest distance along
        Earth's surface between two geographic points. This accounts for Earth's
        oblate shape and provides accuracy suitable for navigation and surveying.

        Args:
            other (GeoPoint): Target point to calculate distance to.

        Returns:
            Meter: Geodesic distance in meters between the two points.

        Example:
            >>> seoul = GeoPoint.from_deg(37.5665, 126.9780)
            >>> busan = GeoPoint.from_deg(35.1796, 129.0756)
            >>> distance = seoul.distance_to(busan)
            >>> print(f"{float(distance)/1000:.1f} km")
            325.4 km
        """
        az12, az21, dist = _WGS84.inv(
            float(self.longitude),
            float(self.latitude),
            float(other.longitude),
            float(other.latitude),
            radians=True,
        )
        return Meter(dist)

    def _move_to(self, azimuth: Angle, distance: Length) -> tuple[Latitude, Longitude]:
        """Move this GeoPoint to a new location in-place.

        Performs a geodesic forward calculation and updates this GeoPoint's
        coordinates to the resulting position. This method modifies the current
        instance rather than creating a new one, making it more efficient for
        performance-critical applications like simulations where minimizing
        object allocation and garbage collection pressure is important.

        Args:
            azimuth (Angle): Bearing angle from North (0°=North, 90°=East, etc.).
            distance (Length): Distance to move along the bearing.
        """
        lon, lat, back_az = _WGS84.fwd(
            float(self.longitude),
            float(self.latitude),
            float(azimuth),
            float(distance),
            radians=True,
        )

        return lat, lon

    def forward(self, azimuth: Angle, distance: Length) -> "GeoPoint":
        """Calculate a new GeoPoint at the given bearing and distance.

        Performs a geodesic forward calculation using the WGS84 ellipsoid to
        determine the geographic coordinates reached by traveling from this
        point along the specified bearing for the given distance.

        This method returns a new GeoPoint without modifying the current instance.
        For in-place modification, use move_to() instead.

        Args:
            azimuth (Angle): Bearing angle from North (0°=North, 90°=East, etc.).
            distance (Length): Distance to travel along the bearing.

        Returns:
            GeoPoint: New geographic point reached by the forward calculation.

        Example:
            >>> seoul = GeoPoint.from_deg(37.5665, 126.9780)
            >>> # Move 1km east from Seoul
            >>> point_east = seoul.forward(Degree(90), Meter(1000))
            >>> print(point_east)
            GeoPoint(latitude=37.5665 °N/S, longitude=126.989 °E/W)
        """
        lon, lat = self._move_to(azimuth, distance)

        latitude = Latitude.from_si(lat)  # lat is already in radians
        longitude = Longitude.from_si(lon)  # lon is already in radians
        return GeoPoint(latitude=latitude, longitude=longitude)

    def move_to(self, azimuth: Angle, distance: Length) -> None:
        """Move this GeoPoint to a new location in-place.

        Performs a geodesic forward calculation and updates this GeoPoint's
        coordinates to the resulting position. This method modifies the current
        instance rather than creating a new one, making it more efficient for
        performance-critical applications like simulations where minimizing
        object allocation and garbage collection pressure is important.

        Args:
            azimuth (Angle): Bearing angle from North (0°=North, 90°=East, etc.).
            distance (Length): Distance to move along the bearing.

        Example:
            >>> point = GeoPoint.from_deg(37.5665, 126.9780)
            >>> print(f"Before: {point}")
            Before: GeoPoint(latitude=37.5665 °N/S, longitude=126.978 °E/W)
            >>> point.move_to(Degree(0), Meter(1000))  # Move 1km north
            >>> print(f"After: {point}")
            After: GeoPoint(latitude=37.5755 °N/S, longitude=126.978 °E/W)
        """
        lon, lat = self._move_to(azimuth, distance)

        self.latitude = Latitude.from_si(lat)  # lat is already in radians
        self.longitude = Longitude.from_si(lon)  # lon is already in radians

    def passed_through_target(
        self, start_point: "GeoPoint", target: "GeoPoint"
    ) -> bool:
        """Check if movement from start_point to current position passed through target.

        This method checks if the target point is within the rectangular bounding box
        formed by the start position and current position. This is useful for discrete
        time simulations where you need to detect if a drone passed through a waypoint
        between simulation steps.

        Args:
            start_point (GeoPoint): Previous position before movement.
            target (GeoPoint): Target point to check for passage.

        Returns:
            bool: True if target is within the bounding box of the movement path.

        Example:
            >>> # Drone movement simulation
            >>> drone = GeoPoint.from_deg(37.5665, 126.9780)
            >>> start_pos = GeoPoint.from_deg(37.5665, 126.9780)  # Save start position
            >>> target = GeoPoint.from_deg(37.5675, 126.9790)    # Waypoint
            >>>
            >>> # Drone moves during simulation step
            >>> drone.move_to(Degree(45), Meter(2000))  # Move northeast 2km
            >>>
            >>> # Check if drone passed through waypoint
            >>> if drone.passed_through_target(start_pos, target):
            ...     print("Waypoint reached!")
            Waypoint reached!
        """
        # Get coordinates as float values for comparison
        start_lat = float(start_point.latitude)
        start_lon = float(start_point.longitude)
        current_lat = float(self.latitude)
        current_lon = float(self.longitude)
        target_lat = float(target.latitude)
        target_lon = float(target.longitude)

        # Check if target is within the bounding box
        lat_in_range = (
            min(start_lat, current_lat) <= target_lat <= max(start_lat, current_lat)
        )
        lon_in_range = (
            min(start_lon, current_lon) <= target_lon <= max(start_lon, current_lon)
        )

        return lat_in_range and lon_in_range
