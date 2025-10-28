"""Advanced vehicle simulation framework with state machine and generic type support.

This package provides a comprehensive, extensible vehicle simulation framework designed for
autonomous vehicle operations in discrete event simulation environments. The architecture
combines abstract base classes with concrete implementations, featuring state machine
integration, generic type safety, and specialized vehicle behaviors.

The framework supports scalable multi-vehicle simulations with realistic operational
constraints, energy management, and mission coordination capabilities suitable for
research, planning, and operational analysis applications.

Core Architecture:
    Three-Tier Design:
        • Vehicle (ABC): Foundation abstract base class with state machine integration
        • Drone[T]: Generic concrete drone implementation with type-safe task management
        • DeliveryDrone: Specialized concrete implementation for package delivery operations

    Design Patterns:
        • Template Method: Vehicle defines structure, subclasses implement specifics
        • State Pattern: State machine-driven behavior with validated transitions
        • Generic Types: Type-safe task assignment with compile-time validation
        • Observer Pattern: Integration with discrete event simulation frameworks

State Machine Integration:
    Built-in Support:
        • StateMachine framework integration for validated state transitions
        • Configurable state graphs with action-based transitions and effects
        • Abstract state management with concrete implementation flexibility

    Extensible State Behaviors:
        • Overrideable state entry methods (enter_*) for transition logic
        • Customizable state behavior handlers (on_*) for ongoing operations
        • Flexible state machine initialization for specialized vehicle types

Generic Type System:
    Type Safety Features:
        • Drone[T] generic implementation with task type constraints
        • Compile-time validation of task assignment operations
        • Specialized implementations with preserved type safety

    Concrete Specializations:
        • DeliveryDrone(Drone[DeliveryTask]): Package delivery optimization
        • Extensible pattern for surveillance, inspection, and rescue operations
        • Custom task types with domain-specific behaviors

Key Capabilities:
    Simulation Features:
        • High-precision geographic positioning with WGS84 coordinate system
        • Comprehensive battery management with multi-phase power consumption
        • Timer management system with automatic lifecycle handling
        • Task queue management with priority-based scheduling

    Performance Optimizations:
        • Memory-efficient design for large-scale fleet simulations
        • Optimized update cycles with configurable granularity
        • Automatic resource cleanup and lifecycle management
        • Scalable architecture supporting hundreds of concurrent vehicles

Integration Dependencies:
    Core Modules:
        • ..state: State machine framework and validation system
        • ..geo: WGS84 geographic coordinate system and navigation
        • ..energy: Battery management and power consumption modeling
        • ..mission: Task assignment framework and execution management
        • ..timer: Time management and scheduling utilities
        • ..unit: Type-safe measurement system and unit conversions

Usage Examples:
    Basic Drone Creation:
        >>> from dronesim.vehicles import Drone
        >>> from dronesim.geo import GeoPoint
        >>> from dronesim.energy.battery import BatteryStatus
        >>> from dronesim.energy.unit import WattHour
        >>> from dronesim.unit import KilometersPerHour, Watt
        >>>
        >>> # Create generic drone with battery configuration
        >>> start_pos = GeoPoint.from_deg(37.5665, 126.9780)  # Seoul coordinates
        >>> battery = BatteryStatus(WattHour(100.0), WattHour(85.0))  # 100Wh capacity, 85Wh current
        >>> drone = Drone(
        ...     pos=start_pos,
        ...     battery=battery,
        ...     velocity=KilometersPerHour(60.0),
        ...     power_transit=Watt(25.0)
        ... )

    Specialized Delivery Operations:
        >>> from dronesim.vehicles import DeliveryDrone
        >>> from dronesim.mission import DeliveryTask
        >>>
        >>> # Create delivery-specialized drone
        >>> delivery_drone = DeliveryDrone(pos=start_pos, battery=battery)
        >>>
        >>> # Type-safe delivery task assignment
        >>> pickup_location = GeoPoint.from_deg(37.5700, 126.9800)
        >>> delivery_location = GeoPoint.from_deg(37.5750, 126.9850)
        >>> delivery_task = DeliveryTask(pickup_location, delivery_location)
        >>> success = delivery_drone.assign(delivery_task)  # Only accepts DeliveryTask

    Custom Vehicle Implementation:
        >>> class SurveillanceDrone(Drone[SurveillanceTask]):
        ...     def vehicle_update(self, dt: Time, now: Time):
        ...         super().vehicle_update(dt, now)
        ...         self._update_surveillance_pattern(dt)
        ...         self._process_sensor_data()
        ...
        ...     def on_navigating(self, dt: Time, now: Time):
        ...         super().on_navigating(dt, now)
        ...         self._maintain_surveillance_altitude()
        ...         self._adjust_camera_orientation()
"""

from .drone import Drone
from .drone_delivery import DeliveryDrone
from .vehicle import Vehicle

__all__ = ["Vehicle", "Drone", "DeliveryDrone"]
