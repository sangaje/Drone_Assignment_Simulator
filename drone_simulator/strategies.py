"""
Assignment strategies for solving drone assignment problems.
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional
import random
import time
from .models import Drone, Task, Assignment, AssignmentResult
from .problem import AssignmentProblem


class AssignmentStrategy(ABC):
    """Base class for assignment strategies."""
    
    @abstractmethod
    def solve(self, problem: AssignmentProblem) -> AssignmentResult:
        """
        Solve the assignment problem.
        
        Args:
            problem: The assignment problem to solve
            
        Returns:
            AssignmentResult containing the solution
        """
        pass
    
    @abstractmethod
    def get_name(self) -> str:
        """Return the name of the strategy."""
        pass


class RandomAssignment(AssignmentStrategy):
    """Randomly assigns tasks to drones."""
    
    def __init__(self, seed: Optional[int] = None):
        """
        Initialize random assignment strategy.
        
        Args:
            seed: Random seed for reproducibility
        """
        self.seed = seed
    
    def get_name(self) -> str:
        return "Random Assignment"
    
    def solve(self, problem: AssignmentProblem) -> AssignmentResult:
        """Randomly assign tasks to available drones."""
        start_time = time.time()
        
        if self.seed is not None:
            random.seed(self.seed)
        
        assignments = []
        unassigned_tasks = []
        
        # Create a copy of drones to track capacity
        drone_states = {drone.drone_id: drone.available_capacity() for drone in problem.drones}
        
        # Shuffle tasks for randomness
        shuffled_tasks = problem.tasks.copy()
        random.shuffle(shuffled_tasks)
        
        for task in shuffled_tasks:
            # Find available drones that can handle the task
            available_drones = [
                drone for drone in problem.drones
                if drone_states[drone.drone_id] >= task.load_requirement
            ]
            
            if available_drones:
                # Pick a random drone
                selected_drone = random.choice(available_drones)
                distance = selected_drone.location.distance_to(task.location)
                cost = distance / task.priority
                
                assignment = Assignment(
                    drone=selected_drone,
                    task=task,
                    distance=distance,
                    cost=cost
                )
                assignments.append(assignment)
                
                # Update drone capacity
                drone_states[selected_drone.drone_id] -= task.load_requirement
            else:
                unassigned_tasks.append(task)
        
        total_cost = sum(a.cost for a in assignments)
        total_distance = sum(a.distance for a in assignments)
        computation_time = time.time() - start_time
        
        return AssignmentResult(
            assignments=assignments,
            total_cost=total_cost,
            total_distance=total_distance,
            unassigned_tasks=unassigned_tasks,
            computation_time=computation_time,
            strategy_name=self.get_name()
        )


class NearestAssignment(AssignmentStrategy):
    """Assigns each task to the nearest available drone (greedy approach)."""
    
    def __init__(self, consider_priority: bool = True):
        """
        Initialize nearest assignment strategy.
        
        Args:
            consider_priority: Whether to consider task priority in assignment
        """
        self.consider_priority = consider_priority
    
    def get_name(self) -> str:
        name = "Nearest Drone Assignment"
        if self.consider_priority:
            name += " (with priority)"
        return name
    
    def solve(self, problem: AssignmentProblem) -> AssignmentResult:
        """Assign each task to the nearest available drone."""
        start_time = time.time()
        
        assignments = []
        unassigned_tasks = []
        
        # Create a copy of drones to track capacity
        drone_states = {drone.drone_id: drone.available_capacity() for drone in problem.drones}
        
        # Sort tasks by priority (highest first) if considering priority
        tasks = sorted(problem.tasks, key=lambda t: t.priority, reverse=True) if self.consider_priority else problem.tasks.copy()
        
        for task in tasks:
            # Find the nearest drone that can handle the task
            best_drone = None
            best_metric = float('inf')
            
            for drone in problem.drones:
                if drone_states[drone.drone_id] >= task.load_requirement:
                    distance = drone.location.distance_to(task.location)
                    
                    # Metric based on distance (and priority if enabled)
                    metric = distance / task.priority if self.consider_priority else distance
                    
                    if metric < best_metric:
                        best_metric = metric
                        best_drone = drone
            
            if best_drone:
                distance = best_drone.location.distance_to(task.location)
                cost = distance / task.priority
                
                assignment = Assignment(
                    drone=best_drone,
                    task=task,
                    distance=distance,
                    cost=cost
                )
                assignments.append(assignment)
                
                # Update drone capacity
                drone_states[best_drone.drone_id] -= task.load_requirement
            else:
                unassigned_tasks.append(task)
        
        total_cost = sum(a.cost for a in assignments)
        total_distance = sum(a.distance for a in assignments)
        computation_time = time.time() - start_time
        
        return AssignmentResult(
            assignments=assignments,
            total_cost=total_cost,
            total_distance=total_distance,
            unassigned_tasks=unassigned_tasks,
            computation_time=computation_time,
            strategy_name=self.get_name()
        )


class MILPAssignment(AssignmentStrategy):
    """
    Solves the assignment problem using Mixed Integer Linear Programming.
    Requires pulp library.
    """
    
    def __init__(self, time_limit: Optional[float] = None):
        """
        Initialize MILP assignment strategy.
        
        Args:
            time_limit: Maximum time in seconds for solver (None for no limit)
        """
        self.time_limit = time_limit
        self._pulp_available = self._check_pulp()
    
    def _check_pulp(self) -> bool:
        """Check if pulp is available."""
        try:
            import pulp
            return True
        except ImportError:
            return False
    
    def get_name(self) -> str:
        return "MILP Optimal Assignment"
    
    def solve(self, problem: AssignmentProblem) -> AssignmentResult:
        """Solve using MILP optimization."""
        if not self._pulp_available:
            raise ImportError(
                "pulp library is required for MILP assignment. "
                "Install it with: pip install pulp"
            )
        
        import pulp
        
        start_time = time.time()
        
        # Create the LP problem
        lp_problem = pulp.LpProblem("Drone_Assignment", pulp.LpMinimize)
        
        # Decision variables: x[i][j] = 1 if drone i is assigned to task j
        x = {}
        for i, drone in enumerate(problem.drones):
            for j, task in enumerate(problem.tasks):
                x[i, j] = pulp.LpVariable(f"x_{i}_{j}", cat='Binary')
        
        # Objective: minimize total cost
        cost_matrix = problem.get_cost_matrix()
        lp_problem += pulp.lpSum(
            cost_matrix[i][j] * x[i, j]
            for i in range(len(problem.drones))
            for j in range(len(problem.tasks))
        )
        
        # Constraint: Each task assigned to at most one drone
        for j in range(len(problem.tasks)):
            lp_problem += pulp.lpSum(x[i, j] for i in range(len(problem.drones))) <= 1
        
        # Constraint: Drone capacity
        for i, drone in enumerate(problem.drones):
            lp_problem += pulp.lpSum(
                problem.tasks[j].load_requirement * x[i, j]
                for j in range(len(problem.tasks))
            ) <= drone.max_capacity
        
        # Solve
        if self.time_limit:
            solver = pulp.PULP_CBC_CMD(timeLimit=self.time_limit, msg=0)
        else:
            solver = pulp.PULP_CBC_CMD(msg=0)
        
        lp_problem.solve(solver)
        
        # Extract solution
        assignments = []
        assigned_task_ids = set()
        
        for i, drone in enumerate(problem.drones):
            for j, task in enumerate(problem.tasks):
                if pulp.value(x[i, j]) == 1:
                    distance = drone.location.distance_to(task.location)
                    cost = distance / task.priority
                    
                    assignment = Assignment(
                        drone=drone,
                        task=task,
                        distance=distance,
                        cost=cost
                    )
                    assignments.append(assignment)
                    assigned_task_ids.add(task.task_id)
        
        unassigned_tasks = [task for task in problem.tasks if task.task_id not in assigned_task_ids]
        
        total_cost = sum(a.cost for a in assignments)
        total_distance = sum(a.distance for a in assignments)
        computation_time = time.time() - start_time
        
        return AssignmentResult(
            assignments=assignments,
            total_cost=total_cost,
            total_distance=total_distance,
            unassigned_tasks=unassigned_tasks,
            computation_time=computation_time,
            strategy_name=self.get_name(),
            metadata={"solver_status": pulp.LpStatus[lp_problem.status]}
        )


class UserDefinedAssignment(AssignmentStrategy):
    """Allows user to define custom assignment mapping."""
    
    def __init__(self, assignment_map: Dict[int, int]):
        """
        Initialize user-defined assignment strategy.
        
        Args:
            assignment_map: Dictionary mapping task_id to drone_id
        """
        self.assignment_map = assignment_map
    
    def get_name(self) -> str:
        return "User-Defined Assignment"
    
    def solve(self, problem: AssignmentProblem) -> AssignmentResult:
        """Apply user-defined assignments."""
        start_time = time.time()
        
        assignments = []
        unassigned_tasks = []
        
        # Create drone lookup
        drone_lookup = {drone.drone_id: drone for drone in problem.drones}
        drone_states = {drone.drone_id: drone.available_capacity() for drone in problem.drones}
        
        for task in problem.tasks:
            if task.task_id in self.assignment_map:
                drone_id = self.assignment_map[task.task_id]
                
                if drone_id in drone_lookup:
                    drone = drone_lookup[drone_id]
                    
                    # Check capacity
                    if drone_states[drone_id] >= task.load_requirement:
                        distance = drone.location.distance_to(task.location)
                        cost = distance / task.priority
                        
                        assignment = Assignment(
                            drone=drone,
                            task=task,
                            distance=distance,
                            cost=cost
                        )
                        assignments.append(assignment)
                        
                        # Update capacity
                        drone_states[drone_id] -= task.load_requirement
                    else:
                        unassigned_tasks.append(task)
                else:
                    unassigned_tasks.append(task)
            else:
                unassigned_tasks.append(task)
        
        total_cost = sum(a.cost for a in assignments)
        total_distance = sum(a.distance for a in assignments)
        computation_time = time.time() - start_time
        
        return AssignmentResult(
            assignments=assignments,
            total_cost=total_cost,
            total_distance=total_distance,
            unassigned_tasks=unassigned_tasks,
            computation_time=computation_time,
            strategy_name=self.get_name()
        )
