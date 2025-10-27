"""Timer utilities for simulation time management and scheduling.

This package provides timing functionality for discrete event simulations,
enabling countdown operations, task scheduling, and temporal state management
within the drone simulation framework.

The timer system integrates seamlessly with the unified unit system to provide
type-safe time calculations and precise countdown functionality. Timers are
automatically managed by Vehicle instances to ensure proper lifecycle control
and prevent resource leaks in long-running simulations.

Components:
    Timer: Countdown timer with duration management and completion detection

Key Features:
    • Type-safe time duration handling with automatic unit conversion
    • Countdown functionality with completion detection
    • Automatic lifecycle management through Vehicle timer system
    • Integration with discrete event simulation frameworks
    • Memory-efficient design with automatic cleanup

Timer Lifecycle:
    1. Creation: Initialize timer with specific duration via Vehicle.create_timer()
    2. Management: Automatic advancement handled by Vehicle._timer_update()
    3. Monitoring: Check completion status via Timer.done property
    4. Cleanup: Automatic removal when timer completes

Common Use Cases:
    • Task scheduling and delay operations
    • State transition timing in vehicle operations
    • Countdown timers for mission phases
    • Temporal coordination between multiple vehicles
    • Performance measurement and timing analysis

Example:
    >>> from app.timer import Timer
    >>> from app.unit import Second, Minute
    >>>
    >>> # Timer creation through vehicle management (recommended)
    >>> delay_timer = vehicle.create_timer(Minute(5))
    >>> print(f"Timer duration: {delay_timer.duration}")
    >>>
    >>> # Check completion during simulation
    >>> if delay_timer.done:
    ...     print("Delay period completed!")

Integration:
    Timers integrate with the simulation framework through Vehicle instances,
    which handle automatic advancement and cleanup. Direct Timer instantiation
    is possible but not recommended for production simulations.
"""

from .timer import Timer

__all__ = ["Timer"]
