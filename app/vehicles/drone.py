"""Drone vehicle implementation for the simulation.

This module contains the Drone class which represents autonomous drone vehicles
in the discrete event simulation. Drones have battery management capabilities,
positioning systems, and can perform various tasks within the simulation environment.
"""

from app.energy.battery import BatteryStatus
from app.geo import GeoPoint
import simpy as sp


class Drone:
    """Represents a drone vehicle in the simulation.

    A Drone is an autonomous vehicle that operates within a SimPy discrete event
    simulation environment. Each drone has a unique identifier, current position,
    and battery status that affects its operational capabilities.

    Attributes:
        env (simpy.Environment): The simulation environment the drone operates in.
        id (int): Unique identifier for this drone instance.
        position (Position): Current geographic position of the drone.
        battery (BatteryStatus): Current battery status and capacity information.
    """

    env: sp.Environment
    id: int
    position: GeoPoint
    battery: BatteryStatus

    def __init__(self, env: sp.Environment, pos: GeoPoint, battery: BatteryStatus):
        """Initialize a new Drone instance.

        Args:
            env (simpy.Environment): The simulation environment for this drone.
            pos (GeoPoint): Initial geographic position of the drone.
            battery (BatteryStatus): Initial battery status of the drone.
        """
        self.env = env
        self.id = id(self)
        self.position = pos
        self.battery = battery

    def move_to_position(self, target: GeoPoint) -> sp.Event:
        """Move the drone to a target position.

        This method creates a simulation event that represents the drone
        moving from its current position to the target position. The movement
        consumes battery energy based on the distance traveled.

        Args:
            target (GeoPoint): The target geographic position to move to.

        Returns:
            simpy.Event: A simulation event that completes when the movement is finished.
        """
        def movement_process():
            # Calculate distance and energy consumption
            distance = self.position.distance_to(target)
            # Simple energy model: 1 Wh per km (this could be made more sophisticated)
            energy_consumption = type(self.battery.current).from_si(float(distance) * 0.001 * 3600)
            
            # Check if we have enough battery
            if not self.battery.consume_energy(energy_consumption):
                raise RuntimeError(f"Drone {self.id} has insufficient battery for movement")
            
            # Simulate movement time (e.g., 1 second per 100 meters)
            movement_time = max(1.0, float(distance) / 100.0)
            yield self.env.timeout(movement_time)
            
            # Update position
            self.position = target
            
        return self.env.process(movement_process())

    def get_status(self) -> dict:
        """Get current drone status information.

        Returns:
            dict: Dictionary containing drone status information including
                 position, battery level, and simulation time.
        """
        return {
            'id': self.id,
            'position': {
                'latitude': float(self.position.latitude),
                'longitude': float(self.position.longitude)
            },
            'battery_percentage': self.battery.percentage,
            'battery_current': str(self.battery.current),
            'battery_capacity': str(self.battery.capacity),
            'simulation_time': self.env.now
        }

    def is_operational(self) -> bool:
        """Check if the drone is operational (has sufficient battery).

        Returns:
            bool: True if the drone has sufficient battery to operate, False otherwise.
        """
        # Consider drone operational if battery is above 5%
        return self.battery.percentage > 5.0
