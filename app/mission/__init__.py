"""Mission management system for drone task coordination.

This package provides a comprehensive mission and task management framework
for autonomous drone operations. It implements state-based task execution
with configurable state transitions, progress tracking, and error handling.

The mission system is built around abstract task definitions that can be
extended for specific mission types like delivery, surveillance, or inspection
tasks. Each task maintains its own state machine and lifecycle management.

Components:
    Task: Abstract base class for all mission tasks
    TaskDelivery: Concrete delivery mission implementation
    DeliveryState: State enumeration for delivery task progression

Key Features:
    • Abstract task interface for extensible mission types
    • State machine-based task progression and validation
    • Configurable state transitions with validation rules
    • Geographic waypoint management for mission planning
    • Task abort and error handling capabilities

Architecture:
    The package uses the State Pattern combined with Template Method to
    provide flexible task definitions while enforcing consistent execution
    patterns across different mission types.

Common Use Cases:
    • Package delivery missions with pickup and dropoff
    • Multi-waypoint surveillance routes
    • Inspection tasks with defined checkpoints
    • Emergency response and rescue operations
    • Agricultural monitoring and data collection

Example:
    >>> from app.mission import TaskDelivery, DeliveryState
    >>> from app.geo import GeoPoint
    >>>
    >>> # Create delivery mission
    >>> pickup = GeoPoint.from_deg(37.5665, 126.9780)  # Seoul
    >>> dropoff = GeoPoint.from_deg(37.5675, 126.9890)  # Destination
    >>> delivery = TaskDelivery(pickup, dropoff)
    >>>
    >>> # Progress through states
    >>> print(f"Initial state: {delivery.current}")  # ASSIGNED
    >>> delivery.next()  # Transition to GO_PICKUP
    >>> print(f"Next state: {delivery.current}")  # GO_PICKUP
"""

from .task import Task
from .task_delivery import DeliveryState, DeliveryTask

__all__ = ["Task", "DeliveryTask", "DeliveryState"]
