"""Base unit system foundation for type-safe physical quantities.

This module provides the fundamental Unit class that serves as the abstract
base for all unit types in the drone simulation system. It implements the
core unit family system using automatic ROOT class assignment, which enables
type-safe operations between compatible units while preventing mixing of
incompatible physical quantities.

The unit system is designed around the concept of "unit families" where each
family represents a distinct physical quantity (length, time, angle, etc.).
Units within the same family can be operated on together, while operations
between different families are prevented at runtime.

Key Concepts:
- ROOT Class: Each unit family has a root class that defines the family
- IS_FAMILY_ROOT: Boolean flag marking the base unit of each family
- Automatic Assignment: ROOT classes are determined automatically via MRO
- Type Safety: Operations are restricted to compatible unit families

Classes:
    Unit: Abstract base class for all unit types with family management.

Example:
    >>> class Length(Unit):
    ...     IS_FAMILY_ROOT = True  # This becomes the ROOT for length units
    >>> class Meter(Length):
    ...     pass  # Automatically gets ROOT = Length
    >>> class Foot(Length):
    ...     pass  # Also gets ROOT = Length
    >>> # Meter and Foot can operate together (same ROOT)
    >>> # But Length units cannot operate with Time units (different ROOT)
"""

from __future__ import annotations

from typing import ClassVar

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
