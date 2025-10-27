"""Timer utilities for simulation time management and countdown operations.

This module provides timer functionality for discrete event simulations,
enabling time-based operations such as countdowns, delays, scheduling,
and temporal state management. The Timer class integrates with the unified
time unit system to provide type-safe and accurate timing operations.

The timer implementation is designed for simulation environments where
precise time tracking and countdown functionality are essential for
coordinating vehicle operations, task scheduling, and event timing.
Timers are managed automatically by Vehicle instances to ensure proper
lifecycle management and prevent resource leaks.

Components:
    Timer: Countdown timer with duration management and completion detection

Integration:
    • Unit System: Uses Time units for type-safe temporal calculations
    • Vehicle System: Managed automatically by Vehicle timer infrastructure
    • Simulation: Integrates with discrete event simulation frameworks
    • Task Management: Enables time-based task scheduling and execution

Architecture:
    • Timers are created via Vehicle.create_timer() for proper management
    • Automatic advancement and cleanup handled by Vehicle._timer_update()
    • Type-safe operations prevent timing errors and unit confusion
    • Memory-efficient design with automatic cleanup of completed timers

Example Usage:
    >>> # Create timer through vehicle management system
    >>> delay_timer = vehicle.create_timer(Second(30.0))
    >>> print(f"Duration: {delay_timer.duration}")
    Duration: 30.0 s
    >>>
    >>> # Timer automatically advances during simulation
    >>> # Check completion status
    >>> if delay_timer.done:
    ...     print("Delay completed!")
    >>>
    >>> # Timers are automatically cleaned up when done
"""

from app.unit import Second, Time

_ZERO_TIME = Second(0.0)


class Timer:
    """Countdown timer for simulation time management and scheduling.

    Provides precise countdown functionality with type-safe time duration
    management. The timer tracks remaining time and provides completion
    detection for simulation events, task scheduling, and temporal operations.

    The Timer class integrates seamlessly with the unified time unit system,
    enabling accurate time calculations while maintaining type safety and
    preventing unit conversion errors in simulation code.

    Note:
        Timer instances should not be reused. Create new Timer instances
        for each timing operation to ensure proper state management and
        avoid timing conflicts in simulation environments.

    Attributes:
        _duration (Time): Current remaining time duration. Decreases as
                         timer advances and reaches zero when complete.

    Lifecycle:
        1. Creation: Initialize with specific duration
        2. Advancement: Automatically updated by simulation framework
        3. Completion: Detected via `done` property
        4. Cleanup: Automatically removed from management systems

    Example:
        >>> from app.unit import Second, Minute
        >>> # Create 5-minute timer via vehicle
        >>> timer = vehicle.create_timer(Minute(5.0))
        >>> print(f"Initial duration: {timer.duration}")
        Initial duration: 300.0 s
        >>>
        >>> # Timer automatically advances during simulation
        >>> # Check completion status
        >>> if timer.done:
        ...     print("Timer completed!")
    """

    _duration: Time

    def __init__(self, duration: Time) -> None:
        """Initialize timer with specified duration.

        Creates a new countdown timer set to the specified duration.
        The timer begins in active state and will count down from
        the provided duration to zero.

        Args:
            duration (Time): Initial countdown duration. Must be
                           non-negative time value. Timer will
                           count down from this value to zero.

        Example:
            >>> timer = Timer(Second(30.0))
            >>> print(timer.duration)
            30.0 s
        """
        self._duration = duration

    @property
    def duration(self) -> Time:
        """Get current remaining duration of the timer.

        Returns:
            Time: Current remaining time duration. Decreases as timer
                 advances and reaches zero when timer is complete.

        Example:
            >>> timer = Timer(Second(45.0))
            >>> print(timer.duration)
            45.0 s
        """
        return self._duration

    @property
    def done(self) -> bool:
        """Check if timer has completed countdown.

        Returns:
            bool: True if timer has reached zero or negative duration,
                 False if time remains. Use this to detect timer
                 completion in simulation loops.

        Example:
            >>> timer = Timer(Second(0.0))
            >>> print(timer.done)
            True
            >>> timer.reset(Second(10.0))
            >>> print(timer.done)
            False
        """
        return self._duration <= _ZERO_TIME

    def _advance(self, delta: Time) -> None:
        """Advance timer by reducing remaining duration.

        Private method that decreases the timer's remaining duration
        by the specified time delta. Used internally by simulation
        frameworks to progress timer state.

        Args:
            delta (Time): Time amount to subtract from remaining duration.
                         Should be positive for normal countdown operation.

        Note:
            This is an internal method. Timer advancement is typically
            handled by simulation framework or parent objects.
        """
        self._duration -= delta

    def reset(self, duration: Time) -> None:
        """Reset timer to new duration, restarting countdown.

        Sets the timer to a new duration value, effectively restarting
        the countdown from the specified time. Any previous remaining
        duration is discarded.

        Args:
            duration (Time): New countdown duration to set. Must be
                           non-negative time value. Timer will count
                           down from this value to zero.

        Example:
            >>> timer = Timer(Second(10.0))
            >>> timer._advance(Second(5.0))  # 5 seconds remaining
            >>> timer.reset(Second(30.0))    # Reset to 30 seconds
            >>> print(timer.duration)
            30.0 s
        """
        self._duration = duration

