"""Angular unit definitions for drone orientation and navigation.

This module provides angular units essential for drone simulation, including
radians and degrees. All angular measurements are internally stored in radians
(the SI base unit) for mathematical consistency, while supporting input and
display in both radians and degrees.

These units are commonly used for:
- Drone orientation (pitch, roll, yaw)
- Navigation headings and bearings
- Rotation angles and angular velocities
- Gimbal positioning and camera angles

Classes:
    Radian: Base angular unit in radians (SI unit).
    Degree: Angular unit in degrees with automatic radian conversion.

Type Aliases:
    Angle: Union type for all angular units (Radian | Degree).

Example:
    >>> heading = Degree(45)  # 45 degrees
    >>> print(heading)  # "45.0 °"
    >>> print(float(heading))  # 0.7854 (radians in SI)
    >>>
    >>> turn_angle = Radian(3.14159)  # π radians
    >>> print(turn_angle.to(Degree))  # 180.0 (degrees)
"""

from __future__ import annotations

from math import pi

from .unit_float import UnitFloat

Number = int | float


class Radian(UnitFloat):
    """Angular unit: Radian (SI base unit for angles).

    The Radian class represents angular measurements in radians, which is
    the SI base unit for angles. One radian is the angle subtended by an
    arc equal in length to the radius of the circle.

    This unit is preferred for mathematical calculations involving
    trigonometric functions, rotational mechanics, and angular velocities.

    Attributes:
        IS_FAMILY_ROOT (bool): True, indicating this is the root angular unit.
        SCALE_TO_SI (float): 1.0, no conversion needed for SI base unit.
        SYMBOL (str): "rad", the standard symbol for radians.

    Example:
        >>> angle = Radian(1.5708)  # π/2 radians (90 degrees)
        >>> print(angle)  # "1.5708 rad"
        >>> print(float(angle))  # 1.5708
    """

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "rad"


class Degree(Radian):
    """Angular unit: Degree (1/360 of a full rotation).

    The Degree class represents angular measurements in degrees, where
    360 degrees equals one full rotation. Values are automatically
    converted to radians for internal storage and calculations.

    This unit is commonly used for:
    - Navigation headings (0-360°)
    - Drone orientation angles
    - User-friendly angle specifications
    - Geographic coordinates and bearings

    Attributes:
        SCALE_TO_SI (float): π/180, conversion factor from degrees to radians.
        SYMBOL (str): "°", the standard symbol for degrees.

    Example:
        >>> bearing = Degree(90)  # 90 degrees (due east)
        >>> print(bearing)  # "90.0 °"
        >>> print(float(bearing))  # 1.5708 (π/2 radians)
        >>> print(bearing.to(Radian))  # 1.5708
    """

    SCALE_TO_SI = pi / 180
    SYMBOL = "°"


# Union type for angle units (defined after classes are created)
Angle = Radian | Degree  # Type alias for any angle unit (radians or degrees)
