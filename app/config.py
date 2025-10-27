"""Configuration variables and type definitions for the drone simulation.

This module contains global configuration variables and type aliases used
throughout the drone assignment simulator. These definitions ensure consistency
in data types and provide centralized configuration management for the
simulation system.

The BASE_TYPE union type defines the acceptable numeric types that can be used
for calculations and data processing throughout the simulation, supporting
both Python native types and NumPy arrays for vectorized operations.
"""

from numpy import ndarray

BASE_TYPE = int | float | ndarray