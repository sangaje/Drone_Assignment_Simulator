"""
Assignment problem definition and utilities.
"""

from typing import List, Optional
import random
from .models import Drone, Task, Location


class AssignmentProblem:
    """
    Defines a drone assignment problem with drones and tasks.
    """
    
    def __init__(self, drones: Optional[List[Drone]] = None, tasks: Optional[List[Task]] = None):
        """
        Initialize an assignment problem.
        
        Args:
            drones: List of available drones
            tasks: List of tasks to be assigned
        """
        self.drones = drones or []
        self.tasks = tasks or []
    
    def add_drone(self, drone: Drone):
        """Add a drone to the problem."""
        self.drones.append(drone)
    
    def add_task(self, task: Task):
        """Add a task to the problem."""
        self.tasks.append(task)
    
    def clear(self):
        """Clear all drones and tasks."""
        self.drones = []
        self.tasks = []
    
    @staticmethod
    def generate_random_problem(
        num_drones: int,
        num_tasks: int,
        area_size: float = 100.0,
        max_capacity: float = 1.0,
        seed: Optional[int] = None
    ) -> 'AssignmentProblem':
        """
        Generate a random assignment problem.
        
        Args:
            num_drones: Number of drones to create
            num_tasks: Number of tasks to create
            area_size: Size of the square area for random locations
            max_capacity: Maximum capacity for drones
            seed: Random seed for reproducibility
            
        Returns:
            AssignmentProblem with randomly generated drones and tasks
        """
        if seed is not None:
            random.seed(seed)
        
        drones = []
        for i in range(num_drones):
            location = Location(
                x=random.uniform(0, area_size),
                y=random.uniform(0, area_size)
            )
            drone = Drone(
                drone_id=i,
                location=location,
                max_capacity=max_capacity
            )
            drones.append(drone)
        
        tasks = []
        for i in range(num_tasks):
            location = Location(
                x=random.uniform(0, area_size),
                y=random.uniform(0, area_size)
            )
            task = Task(
                task_id=i,
                location=location,
                priority=random.uniform(0.5, 2.0),
                load_requirement=random.uniform(0.1, 1.0)
            )
            tasks.append(task)
        
        return AssignmentProblem(drones=drones, tasks=tasks)
    
    def get_distance_matrix(self) -> List[List[float]]:
        """
        Calculate distance matrix between all drones and tasks.
        
        Returns:
            2D list where element [i][j] is distance from drone i to task j
        """
        matrix = []
        for drone in self.drones:
            row = []
            for task in self.tasks:
                distance = drone.location.distance_to(task.location)
                row.append(distance)
            matrix.append(row)
        return matrix
    
    def get_cost_matrix(self) -> List[List[float]]:
        """
        Calculate cost matrix between all drones and tasks.
        Cost is based on distance and task priority.
        
        Returns:
            2D list where element [i][j] is cost for drone i to handle task j
        """
        matrix = []
        for drone in self.drones:
            row = []
            for task in self.tasks:
                distance = drone.location.distance_to(task.location)
                # Cost = distance / priority (higher priority = lower cost)
                cost = distance / task.priority
                row.append(cost)
            matrix.append(row)
        return matrix
    
    def __repr__(self) -> str:
        return f"AssignmentProblem(drones={len(self.drones)}, tasks={len(self.tasks)})"
