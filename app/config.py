"""Global configuration and type definitions for drone simulation framework.

This module provides centralized configuration management and fundamental type
definitions used throughout the autonomous drone assignment simulator. It establishes
consistent data types, numeric precision standards, and core type aliases that
ensure compatibility and performance across all simulation components.

The configuration system supports both development and production environments
with appropriate type safety measures and performance optimizations for
large-scale multi-drone simulations.

Type Definitions:
    BASE_TYPE: Union type defining acceptable numeric types for calculations
               and data processing. Supports Python native types (int, float)
               and NumPy arrays for vectorized mathematical operations and
               high-performance numerical computations.

Key Features:
    • Type consistency across all simulation modules
    • NumPy integration for vectorized operations
    • Memory-efficient data type selections
    • Compatibility with scientific computing libraries
    • Support for both scalar and array-based calculations

Usage:
    The BASE_TYPE union is used throughout the codebase for:
    - Physical quantity calculations (distances, velocities, energies)
    - Mathematical operations in unit conversion systems
    - Array-based batch processing of simulation data
    - Integration with scientific computing and visualization libraries

Example:
    >>> from app.config import BASE_TYPE
    >>> import numpy as np
    >>> # All these types are compatible with BASE_TYPE
    >>> scalar_int: BASE_TYPE = 42
    >>> scalar_float: BASE_TYPE = 3.14159
    >>> array_data: BASE_TYPE = np.array([1.0, 2.0, 3.0])
"""

from numpy import ndarray

BASE_TYPE = int | float | ndarray
