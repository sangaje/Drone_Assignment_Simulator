"""Main entry point for the autonomous drone assignment simulator.

This module serves as the primary entry point for executing comprehensive drone
assignment simulations. It orchestrates the initialization of simulation environments,
creation of autonomous drone fleets, task distribution systems, and manages the
complete simulation lifecycle from startup to results collection.

The simulator provides a realistic discrete event simulation of drone operations
including energy-aware flight planning, geographic positioning with WGS84 coordinates,
battery management with degradation models, and multi-drone task coordination.

Key Components:
    • SimPy-based discrete event simulation engine
    • Multi-drone fleet management and coordination
    • Geographic positioning with real-world coordinate systems
    • Battery-aware energy consumption modeling
    • Task assignment algorithms and optimization
    • Real-time performance monitoring and metrics collection

Simulation Features:
    • Scalable multi-drone operations (supports hundreds of vehicles)
    • Realistic flight dynamics and energy consumption
    • Geographic accuracy with WGS84 ellipsoid calculations
    • Configurable simulation parameters and scenarios
    • Comprehensive logging and performance analysis
    • Extensible architecture for custom vehicle types and behaviors

Usage:
    Run this module directly to start a simulation with default parameters,
    or import specific components for custom simulation scenarios.

Example:
    $ python -m app.main
    # Starts simulation with default configuration
"""
