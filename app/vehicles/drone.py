"""Module defining the Drone vehicle class."""
from app.types import BatteryStatus, Position
import simpy as sp


class Drone:
    """Represents a drone vehicle in the simulation."""

    env: sp.Environment
    id: int
    position: Position
    battery: BatteryStatus

    def __init__(self, env: sp.Environment, pos: Position, battery: BatteryStatus):
        """Initialize Drone with simulation environment, position, and battery status."""
        self.env = env
        self.id = id(self)
        self.position = pos
        self.battery = battery
