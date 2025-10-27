"""Vehicle simulation framework for autonomous drone operations.

This package implements a comprehensive vehicle simulation framework built on
discrete event simulation principles. It provides both abstract base classes
and concrete implementations for various vehicle types, with a focus on
autonomous drone operations and multi-vehicle coordination scenarios.

The architecture follows object-oriented design patterns with clear separation
between abstract vehicle interfaces and concrete implementations, enabling
extensible and maintainable simulation systems.

Architecture:
    Vehicle (ABC): Abstract base class defining the vehicle contract
    Drone: Concrete implementation for autonomous quadcopter operations

    The package uses the Template Method pattern where Vehicle provides
    infrastructure and Drone implements specific behaviors through the
    abstract update() method.

Integration Points:
    • SimPy: Discrete event simulation engine and process management
    • Geo: WGS84 geographic coordinate system for positioning
    • Energy: Battery management and power consumption modeling
    • Unit: Type-safe measurements for time, distance, and energy

Key Features:
    • Abstract base class architecture for extensible vehicle types
    • Real-time geographic positioning with geodesic calculations
    • Energy-aware operation with battery state management
    • Discrete event simulation with time-stepped behavior updates
    • Mission-critical operational status monitoring and safety checks

Simulation Capabilities:
    • Multi-vehicle fleet coordination and management
    • Energy-constrained flight planning and optimization
    • Realistic movement physics and operational constraints
    • Performance analysis and mission effectiveness metrics
    • Scalable scenarios from single drone to large fleet operations

Example:
    >>> from app.vehicles import Drone
    >>> from app.geo import GeoPoint
    >>> from app.energy import BatteryStatus, WattHour
    >>> from app.unit import KilometersPerHour, Watt
    >>>
    >>> # Create drone with battery and position
    >>> start_pos = GeoPoint.from_deg(37.5665, 126.9780)  # Seoul
    >>> battery = BatteryStatus(WattHour(55.5), WattHour(55.5))  # 55.5 Wh battery
    >>> drone = Drone(
    ...     pos=start_pos,
    ...     battery=battery,
    ...     velocity=KilometersPerHour(50),
    ...     power_transit=Watt(15.5)
    ... )
    >>>
    >>> # Check drone status
    >>> print(f"Drone {drone.id} operational: {drone.is_operational()}")
    >>> print(f"Battery: {drone.battery.percentage:.1f}%")
"""

from .drone import Drone
from .vehicle import Vehicle

__all__ = ["Vehicle", "Drone"]
