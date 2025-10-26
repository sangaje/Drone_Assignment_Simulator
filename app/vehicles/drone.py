"""Drone vehicle implementation for the simulation.

This module contains the Drone class which represents autonomous drone vehicles
in the discrete event simulation. Drones have battery management capabilities,
positioning systems, and can perform various tasks within the simulation environment.
"""

from app.types import BatteryStatus, Position
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
    position: Position
    battery: BatteryStatus

    def __init__(self, env: sp.Environment, pos: Position, battery: BatteryStatus):
        """Initialize a new Drone instance.

        Args:
            env (simpy.Environment): The simulation environment for this drone.
            pos (Position): Initial position of the drone.
            battery (BatteryStatus): Initial battery status of the drone.
        """
        self.env = env
        self.id = id(self)
        self.position = pos
        self.battery = battery
