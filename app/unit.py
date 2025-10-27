"""Universal unit definitions and conversion utilities for the simulation.

This module provides a comprehensive set of unit definitions, conversion functions,
and measurement utilities used throughout the drone assignment simulator.
It serves as a central repository for all unit-related operations including
physical quantities, dimensional analysis, and unit conversions.

The module supports various measurement systems and provides standardized
interfaces for working with:
- Distance and spatial measurements (meters, kilometers, feet, etc.)
- Time units (seconds, minutes, hours)
- Speed and velocity units (m/s, km/h, mph, knots)
- Weight and mass units (grams, kilograms, pounds)
- Energy and power units (Watts, Joules, Watt-Hours)
- Angular measurements (degrees, radians)

This centralized approach ensures consistency across all simulation components
and provides type-safe unit conversions throughout the application.
"""

from __future__ import annotations

from math import pi
from typing import ClassVar
from config import BASE_TYPE

Number = int | float


class Unit:
    """Base class for all unit types in the simulation.

    This abstract base class provides the foundation for the unit system,
    implementing automatic ROOT class assignment and basic unit metadata.
    All concrete unit classes should inherit from UnitFloat rather than
    directly from this class.

    Attributes:
        ROOT (ClassVar[type[Unit]]): Root class defining the unit family.
        SYMBOL (ClassVar[str]): Unit symbol for display purposes.
        IS_FAMILY_ROOT (ClassVar[bool]): Indicates if this class is a root unit.
    """

    __slots__ = ()
    __array_priority__ = 1000

    ROOT: ClassVar[type[Unit]]
    SYMBOL: ClassVar[str] = ""
    IS_FAMILY_ROOT: ClassVar[bool] = False

    def __init_subclass__(cls, **kwargs):
        """Automatically set ROOT class for subclasses.

        This method is called when a class is subclassed and automatically
        determines the ROOT class by finding the first ancestor with
        IS_FAMILY_ROOT=True, or defaults to self if none found.

        Args:
            **kwargs: Additional keyword arguments passed to super().__init_subclass__.
        """
        super().__init_subclass__(**kwargs)
        if "ROOT" in cls.__dict__ and cls.ROOT is not None:
            return

        if cls.__dict__.get("IS_FAMILY_ROOT", False):
            cls.ROOT = cls
            return

        for base in cls.mro()[1:]:
            if base.__dict__.get("IS_FAMILY_ROOT", False):
                cls.ROOT = base
                return

        cls.ROOT = cls

    @classmethod
    def _check_same_root(cls, unit_type: type[Unit]):
        """Check if two unit types belong to the same physical quantity family.

        This method ensures type safety by verifying that operations are only
        performed between units of the same physical quantity (e.g., length
        units with length units, angle units with angle units).

        Args:
            unit_type: The other unit type to check compatibility with.

        Raises:
            TypeError: If the units belong to different physical quantity families.
        """
        if cls.ROOT is not unit_type.ROOT:
            msg = f"TypeError: {cls.ROOT, unit_type.ROOT}"
            raise TypeError(msg)

class Quantity(Unit):
    """Base class for physical quantities in the simulation.

    This class serves as an abstract base for physical quantity types
    that don't require automatic conversion to float values. It inherits
    the ROOT assignment and type checking capabilities from Unit while
    allowing for more complex quantity representations.
    """
    BASE_TYPE : ClassVar[type] = float
    SCALE_TO_SI: ClassVar[float] = 1.0
    IS_FAMILY_ROOT: ClassVar[bool] = True

    def __new__(cls, value: Number):
        """Create a new UnitFloat instance with automatic SI conversion.

        Args:
            value: Numeric value in the unit's native scale.

        Returns:
            UnitFloat: New instance with value stored in SI units.
        """
        si_val = cls.BASE_TYPE(value) * cls.SCALE_TO_SI
        return cls.BASE_TYPE.__new__(cls, si_val)

    @classmethod
    def from_si(cls, si_value: float) -> UnitFloat:
        """Create instance directly from SI unit value.

        Args:
            si_value: Value already in SI units.

        Returns:
            UnitFloat: New instance with the SI value.
        """
        return float.__new__(cls, si_value)

    def to(self, unit_type: type[UnitFloat]) -> float:
        """Convert to another unit of the same family.

        Args:
            unit_type: Target unit type to convert to.

        Returns:
            float: Value in the target unit's scale.
        """
        self._check_same_root(unit_type)
        return float(self) / unit_type.SCALE_TO_SI

    def as_unit(self, unit_type: type[UnitFloat]) -> UnitFloat:
        """Convert to another unit while preserving type information.

        Args:
            unit_type: Target unit type to convert to.

        Returns:
            UnitFloat: New instance of the target unit type.
        """
        self._check_same_root(unit_type)
        return unit_type.from_si(float(self))

    # -------------------------------- Arithmetic Operations --------------------------------
    def __add__(self, other: UnitFloat) -> UnitFloat:
        """Add two units of the same family.

        Args:
            other: Unit value to add to this unit.

        Returns:
            UnitFloat: Sum of the two units.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return type(self).from_si(float(self) + float(other))

    def __radd__(self, other: UnitFloat) -> UnitFloat:
        """Right-side addition for units.

        Args:
            other: Unit value to add to this unit.

        Returns:
            UnitFloat: Sum of the two units.
        """
        return self.__add__(other)

    def __sub__(self, other: UnitFloat) -> UnitFloat:
        """Subtract two units of the same family.

        Args:
            other: Unit value to subtract from this unit.

        Returns:
            UnitFloat: Difference of the two units.
        """
        self._check_same_root(type(other))
        return type(self).from_si(float(self) - float(other))

    def __rsub__(self, other: UnitFloat) -> UnitFloat:
        """Right-side subtraction for units.

        Args:
            other: Unit value that this unit is subtracted from.

        Returns:
            UnitFloat: Difference with other as minuend.
        """
        self._check_same_root(type(other))
        return type(self).from_si(float(other) - float(self))

    def __mul__(self, k: Number) -> UnitFloat:
        """Multiply unit by scalar value.

        Args:
            k: Numeric scalar to multiply by.

        Returns:
            UnitFloat: Unit scaled by the factor.

        Raises:
            TypeError: If k is not a numeric type.
        """
        if isinstance(k, Number):
            return type(self).from_si(float(self) * float(k))
        raise TypeError

    def __rmul__(self, k: Number) -> UnitFloat:
        """Right-side multiplication by scalar.

        Args:
            k: Numeric scalar to multiply by.

        Returns:
            UnitFloat: Unit scaled by the factor.
        """
        return self.__mul__(k)

    def __truediv__(self, k: Number) -> UnitFloat:
        """Divide unit by scalar value.

        Args:
            k: Numeric scalar to divide by.

        Returns:
            UnitFloat: Unit divided by the scalar.

        Raises:
            TypeError: If k is not a numeric type.
        """
        if isinstance(k, Number):
            return type(self).from_si(float(self) / float(k))
        raise TypeError

    def __iadd__(self, other: UnitFloat) -> UnitFloat:
        """In-place addition with another unit of the same family.

        Args:
            other: Unit value to add to this unit.

        Returns:
            UnitFloat: This unit modified by addition.
        """
        self._check_same_root(type(other))
        return type(self).from_si(float(self) + float(other))

    def __isub__(self, other: UnitFloat) -> UnitFloat:
        """In-place subtraction with another unit of the same family.

        Args:
            other: Unit value to subtract from this unit.

        Returns:
            UnitFloat: This unit modified by subtraction.
        """
        self._check_same_root(type(other))
        return type(self).from_si(float(self) - float(other))

    def __imul__(self, k: Number) -> UnitFloat:
        """In-place multiplication with a scalar value.

        Args:
            k: Numeric scalar to multiply by.

        Returns:
            UnitFloat: This unit scaled by the factor.

        Raises:
            TypeError: If k is not a numeric type.
        """
        if isinstance(k, Number):
            return type(self).from_si(float(self) * float(k))
        raise TypeError

    def __itruediv__(self, k: Number) -> UnitFloat:
        """In-place division with a scalar value.

        Args:
            k: Numeric scalar to divide by.

        Returns:
            UnitFloat: This unit divided by the scalar.

        Raises:
            TypeError: If k is not a numeric type.
        """
        if isinstance(k, Number):
            return type(self).from_si(float(self) / float(k))
        raise TypeError

    def __lt__(self, other: UnitFloat) -> bool:
        """Less-than comparison between two units of the same family.

        Args:
            other: Unit value to compare against.

        Returns:
            bool: True if this unit is less than the other.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return float(self) < float(other)

    def __le__(self, other: UnitFloat) -> bool:
        """Less-than-or-equal comparison between two units of the same family.

        Args:
            other: Unit value to compare against.

        Returns:
            bool: True if this unit is less than or equal to the other.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return float(self) <= float(other)

    def __gt__(self, other: UnitFloat) -> bool:
        """Greater-than comparison between two units of the same family.

        Args:
            other: Unit value to compare against.

        Returns:
            bool: True if this unit is greater than the other.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return float(self) > float(other)

    def __ge__(self, other: UnitFloat) -> bool:
        """Greater-than-or-equal comparison between two units of the same family.

        Args:
            other: Unit value to compare against.

        Returns:
            bool: True if this unit is greater than or equal to the other.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return float(self) >= float(other)

    def __eq__(self, other: UnitFloat) -> bool:
        """Equality comparison between two units of the same family.

        Args:
            other: Unit value to compare against.

        Returns:
            bool: True if this unit equals the other.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return float(self) == float(other)

    def __ne__(self, other: UnitFloat) -> bool:
        """Not-equal comparison between two units of the same family.

        Args:
            other: Unit value to compare against.

        Returns:
            bool: True if this unit does not equal the other.

        Raises:
            TypeError: If units are not from the same family.
        """
        self._check_same_root(type(other))
        return float(self) != float(other)

    def __str__(self) -> str:
        """Return human-readable string representation in the unit's native scale.

        Returns:
            str: Value and symbol in the unit's natural scale (e.g., "90.0 °").
        """
        return f"{self.to(type(self))} {type(self).SYMBOL}".strip()

    def __repr__(self) -> str:
        """Return detailed string representation showing both native and SI values.

        Returns:
            str: Value in native scale with SI equivalent (e.g., "90 ° (= 1.5708 SI)").
        """
        return f"{self.to(type(self)):g} {type(self).SYMBOL} (= {float(self):g} SI)"

class UnitFloat(float, Unit):
    """Base class for type-safe unit calculations with automatic SI conversion.

    This class stores values internally in SI units while allowing operations
    only between compatible unit types (same 'root' family). It provides
    automatic conversion between different units of the same physical quantity.

    Attributes:
        ROOT (ClassVar[type[UnitFloat]]): Root class defining the unit family.
        SCALE_TO_SI (ClassVar[float]): Conversion factor to SI units.
        SYMBOL (ClassVar[str]): Unit symbol for display purposes.
        IS_FAMILY_ROOT (ClassVar[bool]): Indicates if this class is a root unit.
    """

    BASE_TYPE: ClassVar[BASE_TYPE] = float
    SCALE_TO_SI: ClassVar[float] = 1.0
    IS_FAMILY_ROOT: ClassVar[bool] = True

    


# ------------------------------------- Angle -------------------------------------
class Radian(UnitFloat):
    """Angular unit: Radian (SI base unit for angles)."""

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "rad"


class Degree(Radian):
    """Angular unit: Degree (1/360 of a full rotation)."""

    SCALE_TO_SI = pi / 180
    SYMBOL = "°"


# Union type for angle units (defined after classes are created)
Angle = Radian | Degree  # Type alias for any angle unit (radians or degrees)
# ---------------------------------------------------------------------------------


# ----------------------------------- Distance ------------------------------------
class Meter(UnitFloat):
    """Distance unit: Meter (SI base unit for length)."""

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "m"


class Kilometer(Meter):
    """Distance unit: Kilometer (1000 meters)."""

    SCALE_TO_SI = 1000.0
    SYMBOL = "km"


Length = Meter | Kilometer  # Type alias for any length unit

# ---------------------------------------------------------------------------------


# ------------------------------------ Time --------------------------------------
class Second(UnitFloat):
    """Time unit: Second (SI base unit for time)."""

    IS_FAMILY_ROOT = True
    SCALE_TO_SI = 1.0
    SYMBOL = "s"


class Minute(Second):
    """Time unit: Minute (60 seconds)."""

    SCALE_TO_SI = 60.0
    SYMBOL = "min"


class Hour(Second):
    """Time unit: Hour (3600 seconds)."""

    SCALE_TO_SI = 3600.0
    SYMBOL = "h"


Time = Second | Minute | Hour  # Type alias for any time unit

# ---------------------------------------------------------------------------------
