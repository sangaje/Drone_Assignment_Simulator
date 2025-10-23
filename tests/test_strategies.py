"""
Tests for assignment problem and strategies.
"""

import unittest
from drone_simulator.problem import AssignmentProblem
from drone_simulator.models import Drone, Task, Location
from drone_simulator.strategies import (
    RandomAssignment,
    NearestAssignment,
    UserDefinedAssignment
)


class TestAssignmentProblem(unittest.TestCase):
    """Test AssignmentProblem class."""
    
    def test_create_empty_problem(self):
        """Test creating an empty problem."""
        problem = AssignmentProblem()
        self.assertEqual(len(problem.drones), 0)
        self.assertEqual(len(problem.tasks), 0)
    
    def test_add_drone_and_task(self):
        """Test adding drones and tasks."""
        problem = AssignmentProblem()
        
        drone = Drone(drone_id=1, location=Location(0, 0))
        problem.add_drone(drone)
        self.assertEqual(len(problem.drones), 1)
        
        task = Task(task_id=1, location=Location(5, 5))
        problem.add_task(task)
        self.assertEqual(len(problem.tasks), 1)
    
    def test_generate_random_problem(self):
        """Test generating a random problem."""
        problem = AssignmentProblem.generate_random_problem(
            num_drones=5,
            num_tasks=10,
            area_size=100.0,
            seed=42
        )
        self.assertEqual(len(problem.drones), 5)
        self.assertEqual(len(problem.tasks), 10)
    
    def test_distance_matrix(self):
        """Test distance matrix calculation."""
        problem = AssignmentProblem()
        
        drone1 = Drone(drone_id=0, location=Location(0, 0))
        drone2 = Drone(drone_id=1, location=Location(10, 0))
        problem.add_drone(drone1)
        problem.add_drone(drone2)
        
        task1 = Task(task_id=0, location=Location(3, 4))
        task2 = Task(task_id=1, location=Location(13, 4))
        problem.add_task(task1)
        problem.add_task(task2)
        
        matrix = problem.get_distance_matrix()
        self.assertEqual(len(matrix), 2)  # 2 drones
        self.assertEqual(len(matrix[0]), 2)  # 2 tasks
        self.assertEqual(matrix[0][0], 5.0)  # drone1 to task1


class TestRandomAssignment(unittest.TestCase):
    """Test RandomAssignment strategy."""
    
    def test_random_assignment(self):
        """Test random assignment strategy."""
        problem = AssignmentProblem.generate_random_problem(
            num_drones=3,
            num_tasks=5,
            seed=42
        )
        
        strategy = RandomAssignment(seed=42)
        result = strategy.solve(problem)
        
        self.assertIsNotNone(result)
        self.assertEqual(result.strategy_name, "Random Assignment")
        self.assertGreater(len(result.assignments), 0)
    
    def test_random_assignment_respects_capacity(self):
        """Test that random assignment respects drone capacity."""
        problem = AssignmentProblem()
        
        # Create a drone with limited capacity
        drone = Drone(drone_id=0, location=Location(0, 0), max_capacity=1.0)
        problem.add_drone(drone)
        
        # Create tasks that exceed capacity
        for i in range(3):
            task = Task(task_id=i, location=Location(i, i), load_requirement=0.5)
            problem.add_task(task)
        
        strategy = RandomAssignment(seed=42)
        result = strategy.solve(problem)
        
        # Should only assign 2 tasks (total load = 1.0)
        self.assertLessEqual(len(result.assignments), 2)
        self.assertGreater(len(result.unassigned_tasks), 0)


class TestNearestAssignment(unittest.TestCase):
    """Test NearestAssignment strategy."""
    
    def test_nearest_assignment(self):
        """Test nearest assignment strategy."""
        problem = AssignmentProblem()
        
        # Create drones
        drone1 = Drone(drone_id=0, location=Location(0, 0))
        drone2 = Drone(drone_id=1, location=Location(10, 10))
        problem.add_drone(drone1)
        problem.add_drone(drone2)
        
        # Create tasks
        task1 = Task(task_id=0, location=Location(1, 1))  # Near drone1
        task2 = Task(task_id=1, location=Location(11, 11))  # Near drone2
        problem.add_task(task1)
        problem.add_task(task2)
        
        strategy = NearestAssignment()
        result = strategy.solve(problem)
        
        self.assertEqual(len(result.assignments), 2)
        
        # Check that each task is assigned to its nearest drone
        for assignment in result.assignments:
            if assignment.task.task_id == 0:
                self.assertEqual(assignment.drone.drone_id, 0)
            elif assignment.task.task_id == 1:
                self.assertEqual(assignment.drone.drone_id, 1)
    
    def test_nearest_with_priority(self):
        """Test nearest assignment with priority consideration."""
        problem = AssignmentProblem()
        
        drone = Drone(drone_id=0, location=Location(0, 0))
        problem.add_drone(drone)
        
        task1 = Task(task_id=0, location=Location(5, 5), priority=1.0)
        task2 = Task(task_id=1, location=Location(3, 3), priority=2.0)  # Higher priority
        problem.add_task(task1)
        problem.add_task(task2)
        
        strategy = NearestAssignment(consider_priority=True)
        result = strategy.solve(problem)
        
        # Higher priority task should be assigned first
        self.assertEqual(result.assignments[0].task.task_id, 1)


class TestUserDefinedAssignment(unittest.TestCase):
    """Test UserDefinedAssignment strategy."""
    
    def test_user_defined_assignment(self):
        """Test user-defined assignment strategy."""
        problem = AssignmentProblem()
        
        drone1 = Drone(drone_id=0, location=Location(0, 0))
        drone2 = Drone(drone_id=1, location=Location(10, 10))
        problem.add_drone(drone1)
        problem.add_drone(drone2)
        
        task1 = Task(task_id=0, location=Location(1, 1))
        task2 = Task(task_id=1, location=Location(11, 11))
        problem.add_task(task1)
        problem.add_task(task2)
        
        # Define custom mapping
        assignment_map = {0: 1, 1: 0}  # task 0 to drone 1, task 1 to drone 0
        
        strategy = UserDefinedAssignment(assignment_map)
        result = strategy.solve(problem)
        
        self.assertEqual(len(result.assignments), 2)
        
        # Verify custom mapping
        for assignment in result.assignments:
            expected_drone_id = assignment_map[assignment.task.task_id]
            self.assertEqual(assignment.drone.drone_id, expected_drone_id)
    
    def test_user_defined_with_invalid_mapping(self):
        """Test user-defined assignment with invalid drone ID."""
        problem = AssignmentProblem()
        
        drone = Drone(drone_id=0, location=Location(0, 0))
        problem.add_drone(drone)
        
        task = Task(task_id=0, location=Location(1, 1))
        problem.add_task(task)
        
        # Define invalid mapping (drone 99 doesn't exist)
        assignment_map = {0: 99}
        
        strategy = UserDefinedAssignment(assignment_map)
        result = strategy.solve(problem)
        
        self.assertEqual(len(result.assignments), 0)
        self.assertEqual(len(result.unassigned_tasks), 1)


class TestMILPAssignment(unittest.TestCase):
    """Test MILPAssignment strategy."""
    
    def setUp(self):
        """Check if pulp is available."""
        try:
            import pulp
            self.pulp_available = True
        except ImportError:
            self.pulp_available = False
    
    def test_milp_assignment_basic(self):
        """Test MILP assignment on a simple problem."""
        if not self.pulp_available:
            self.skipTest("pulp library not available")
        
        from drone_simulator.strategies import MILPAssignment
        
        problem = AssignmentProblem()
        
        # Create drones
        drone1 = Drone(drone_id=0, location=Location(0, 0), max_capacity=2.0)
        drone2 = Drone(drone_id=1, location=Location(10, 10), max_capacity=2.0)
        problem.add_drone(drone1)
        problem.add_drone(drone2)
        
        # Create tasks
        task1 = Task(task_id=0, location=Location(1, 1), load_requirement=1.0)
        task2 = Task(task_id=1, location=Location(11, 11), load_requirement=1.0)
        problem.add_task(task1)
        problem.add_task(task2)
        
        strategy = MILPAssignment()
        result = strategy.solve(problem)
        
        self.assertEqual(len(result.assignments), 2)
        self.assertEqual(len(result.unassigned_tasks), 0)
        self.assertEqual(result.metadata.get('solver_status'), 'Optimal')
    
    def test_milp_respects_capacity(self):
        """Test that MILP respects capacity constraints."""
        if not self.pulp_available:
            self.skipTest("pulp library not available")
        
        from drone_simulator.strategies import MILPAssignment
        
        problem = AssignmentProblem()
        
        # Create a drone with limited capacity
        drone = Drone(drone_id=0, location=Location(0, 0), max_capacity=1.0)
        problem.add_drone(drone)
        
        # Create tasks that exceed capacity if all assigned
        for i in range(3):
            task = Task(task_id=i, location=Location(i, i), load_requirement=0.5)
            problem.add_task(task)
        
        strategy = MILPAssignment()
        result = strategy.solve(problem)
        
        # Should only assign 2 tasks (total load = 1.0)
        self.assertLessEqual(len(result.assignments), 2)
        
        # Verify capacity constraint
        total_load = sum(a.task.load_requirement for a in result.assignments)
        self.assertLessEqual(total_load, 1.0)


if __name__ == '__main__':
    unittest.main()
