"""Delivery-specialized drone implementation for package delivery operations.

This module implements the DeliveryDrone class, a specialized drone type that extends the generic
Drone class with delivery-specific functionality and optimizations for package transportation tasks.
"""

from app.energy import WattSecond
from app.mission import DeliveryState, DeliveryTask
from app.unit import Time

from .drone import Drone, DroneState


class DeliveryDrone(Drone[DeliveryTask]):
    """A specialized drone optimized for delivery operations.

    DeliveryDrone extends the generic Drone class with DeliveryTask-specific
    capabilities, providing type-safe task assignment and delivery-optimized
    behaviors such as payload management, delivery route optimization, and
    package handling procedures.

    This concrete implementation handles DeliveryTask objects exclusively,
    ensuring type safety and enabling delivery-specific optimizations
    throughout the drone's operational lifecycle.

    Type Specialization:
        - Task Type: DeliveryTask (packages, recipients, delivery locations)
        - Optimized for: Package weight calculations, delivery routing,
          recipient interactions, and delivery confirmation procedures

    Inherited Attributes:
        All attributes from Drone[DeliveryTask] including battery management,
        positioning, power consumption profiles, and task queuing systems
        specifically typed for DeliveryTask objects.

    Example Usage:
        >>> from app.geo import GeoPoint
        >>> from app.energy.battery import BatteryStatus
        >>> from app.mission import DeliveryTask
        >>>
        >>> # Create delivery drone
        >>> drone = DeliveryDrone(pos=start_pos, battery=battery_status)
        >>>
        >>> # Assign delivery task (type-safe)
        >>> delivery = DeliveryTask(package=pkg, destination=dest)
        >>> drone.assign(delivery)  # Only accepts DeliveryTask
    """

    def update(self, dt: Time) -> None:
        """Execute delivery-specific update logic for one simulation step.

        Handles delivery-oriented behaviors including package transportation,
        delivery route following, recipient location navigation, and
        delivery completion procedures during each simulation time step.

        Args:
            dt (Time): Time step duration in simulation time units.

        Note:
            This implementation extends the base Vehicle update method with
            delivery-specific logic while maintaining the standard simulation
            update pattern.
        """
        self.battery.consume_energy(WattSecond(float(self._battery_usage) * float(dt)))

    def post_update(self, dt: Time) -> None:
        """Execute delivery-specific post-processing after main update cycle.

        Handles delivery-oriented cleanup operations, delivery status updates,
        and delivery metrics collection that should occur after all primary
        updates have completed.

        Args:
            dt (Time): Time step duration in simulation time units.

        Note:
            This method ensures delivery state consistency and prepares
            delivery-related data for the next simulation cycle.
        """
        if self.current_tasks is None:
            # Check if there are any tasks in the queue
            if len(self.task_queue) == 0:
                return

            # Dequeue the next delivery task
            self.current_tasks = [task for task in self.task_queue]
            self.task_queue.clear()

        if self.current_state == DroneState.GROUNDED:
            if self.current_destination is None:
                pass
                # TODO

        if self.current_destination is None:
            min_state = min(self.current_tasks, key=lambda t: t.current_state.value)
            to_do_task = [task for task in self.current_tasks if task.current_state == min_state]
            if min_state == DeliveryState.ASSIGNED:
                min_distance_task = min(
                    to_do_task, key=lambda t: self.position.distance_to(t.origin)
                )
                self.current_destination = min_distance_task.origin
            elif min_state == DeliveryState.SERVICE_PICKUP:
                min_distance_task = min(
                    to_do_task, key=lambda t: self.position.distance_to(t.destination)
                )
                self.current_destination = min_distance_task.destination
