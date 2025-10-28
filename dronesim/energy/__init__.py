"""Energy management system for drone simulation.

This package provides comprehensive energy and battery management functionality
for drone simulation, including battery models, energy units, charging/discharging
algorithms, and power consumption tracking.

The package combines type-safe energy units with realistic battery behavior
models to enable accurate simulation of drone energy consumption, flight time
estimation, and mission planning based on available battery capacity.

Components:
    Battery Management:
        BatteryStatus: Battery state tracking and energy management

    Energy Units:
        WattSecond: SI base unit for energy (Joules)
        WattMinute: Energy in Watt-Minutes (60 Joules)
        WattHour: Energy in Watt-Hours (3600 Joules)
        KilowattHour: Energy in Kilowatt-Hours (3.6M Joules)
        Energy: Type alias for energy unit family

Key Features:
    • Type-safe energy calculations with automatic unit conversion
    • Realistic battery charging and discharging simulation
    • Battery capacity degradation modeling over time
    • Energy consumption prediction for flight planning
    • Battery health monitoring and diagnostics
    • Temperature effects on battery performance

Common Use Cases:
    • Flight time estimation based on battery capacity
    • Mission planning with energy constraints
    • Battery health monitoring and maintenance
    • Energy optimization for extended flight operations
    • Charging station management and scheduling

Example:
    >>> from dronesim.energy import BatteryStatus, WattHour, WattSecond
    >>> from dronesim.unit import Second
    >>>
    >>> # Create a drone battery (5000 mAh at 11.1V ≈ 55.5 Wh)
    >>> battery = BatteryStatus(WattHour(55.5), WattHour(55.5))  # Capacity, current
    >>> print(f"Full battery: {battery.percentage:.1f}%")
    >>>
    >>> # Simulate flight energy consumption
    >>> power_draw = 15.5  # 15.5W power draw
    >>> flight_time = 300  # 5 minutes in seconds
    >>> energy_used = WattHour(power_draw * flight_time / 3600)  # Convert to Wh
    >>>
    >>> # Consume energy and check remaining capacity
    >>> success = battery.consume_energy(energy_used)
    >>> print(f"Energy consumed: {success}")
    >>> print(f"Remaining battery: {battery.percentage:.1f}%")
"""

from .battery import BatteryStatus
from .unit import Energy, KilowattHour, WattHour, WattMinute, WattSecond

EnergyStatus = BatteryStatus

__all__ = [
    "BatteryStatus",
    "WattHour",
    "WattMinute",
    "WattSecond",
    "KilowattHour",
    "Energy",
    "EnergyStatus",
]
