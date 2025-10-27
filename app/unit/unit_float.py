"""Float-based unit system with automatic SI conversion and type safety.

This module provides the UnitFloat class, which serves as the foundation for
all numeric unit types in the simulation. It combines Python's float type
with unit safety, automatic SI conversion, and type checking to prevent
mixing incompatible units.

Key Features:
- Automatic conversion to SI units for internal storage
- Type-safe operations between compatible unit families
- Arithmetic operations with scalar values and other units
- Conversion methods between different units of the same family
- Human-readable string representations

Classes:
    UnitFloat: Base class for all float-based units with automatic SI conversion.

Example:
    >>> class Meter(UnitFloat):
    ...     IS_FAMILY_ROOT = True
    ...     SCALE_TO_SI = 1.0
    ...     SYMBOL = "m"
    ...
    >>> class Kilometer(Meter):
    ...     SCALE_TO_SI = 1000.0
    ...     SYMBOL = "km"
    ...
    >>> distance = Kilometer(5.2)  # 5.2 km
    >>> print(distance)  # "5.2 km"
    >>> print(float(distance))  # 5200.0 (meters in SI)
"""
from __future__ import annotations

from typing import ClassVar

from .unit_base import Unit

Number = int | float

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

    SCALE_TO_SI: ClassVar[float] = 1.0
    IS_FAMILY_ROOT: ClassVar[bool] = True

    def __new__(cls, value: Number):
        """Create a new UnitFloat instance with automatic SI conversion.

        Args:
            value: Numeric value in the unit's native scale.

        Returns:
            UnitFloat: New instance with value stored in SI units.
        """
        si_val = float(value) * cls.SCALE_TO_SI
        return float.__new__(cls, si_val)

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
