"""
Core data models for the drone assignment simulator.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import math


@dataclass
class Location:
    """Represents a 2D location with x, y coordinates."""
    x: float
    y: float
    
    def distance_to(self, other: 'Location') -> float:
        """Calculate Euclidean distance to another location."""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
    
    def __repr__(self) -> str:
        return f"Location({self.x:.2f}, {self.y:.2f})"


@dataclass
class Drone:
    """Represents a drone with ID, location, and capacity."""
    drone_id: int
    location: Location
    max_capacity: float = 1.0
    current_load: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def available_capacity(self) -> float:
        """Return the remaining capacity of the drone."""
        return self.max_capacity - self.current_load
    
    def can_handle_task(self, task: 'Task') -> bool:
        """Check if the drone can handle a task based on capacity."""
        return self.available_capacity() >= task.load_requirement
    
    def __repr__(self) -> str:
        return f"Drone(id={self.drone_id}, loc={self.location}, capacity={self.current_load}/{self.max_capacity})"


@dataclass
class Task:
    """Represents a task to be assigned to a drone."""
    task_id: int
    location: Location
    priority: float = 1.0
    load_requirement: float = 1.0
    deadline: Optional[float] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def __repr__(self) -> str:
        return f"Task(id={self.task_id}, loc={self.location}, priority={self.priority})"


@dataclass
class Assignment:
    """Represents an assignment of a task to a drone."""
    drone: Drone
    task: Task
    distance: float
    cost: float = 0.0
    
    def __repr__(self) -> str:
        return f"Assignment(drone={self.drone.drone_id}, task={self.task.task_id}, dist={self.distance:.2f})"


@dataclass
class AssignmentResult:
    """Contains the results of an assignment solution."""
    assignments: list[Assignment]
    total_cost: float
    total_distance: float
    unassigned_tasks: list[Task]
    computation_time: float
    strategy_name: str
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def get_summary(self) -> Dict[str, Any]:
        """Get a summary of the assignment result."""
        return {
            "strategy": self.strategy_name,
            "num_assignments": len(self.assignments),
            "num_unassigned": len(self.unassigned_tasks),
            "total_cost": self.total_cost,
            "total_distance": self.total_distance,
            "computation_time": self.computation_time,
            "average_distance": self.total_distance / len(self.assignments) if self.assignments else 0,
        }
    
    def __repr__(self) -> str:
        return (f"AssignmentResult(strategy={self.strategy_name}, "
                f"assignments={len(self.assignments)}, "
                f"unassigned={len(self.unassigned_tasks)}, "
                f"cost={self.total_cost:.2f})")
