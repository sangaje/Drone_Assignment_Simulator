"""
Tests for drone simulator models.
"""

import unittest
from drone_simulator.models import Location, Drone, Task, Assignment, AssignmentResult


class TestLocation(unittest.TestCase):
    """Test Location class."""
    
    def test_distance_calculation(self):
        """Test distance calculation between two locations."""
        loc1 = Location(0, 0)
        loc2 = Location(3, 4)
        self.assertEqual(loc1.distance_to(loc2), 5.0)
    
    def test_distance_same_location(self):
        """Test distance to same location is zero."""
        loc = Location(5, 5)
        self.assertEqual(loc.distance_to(loc), 0.0)
    
    def test_distance_symmetry(self):
        """Test that distance is symmetric."""
        loc1 = Location(1, 2)
        loc2 = Location(4, 6)
        self.assertEqual(loc1.distance_to(loc2), loc2.distance_to(loc1))


class TestDrone(unittest.TestCase):
    """Test Drone class."""
    
    def test_drone_creation(self):
        """Test creating a drone."""
        loc = Location(10, 20)
        drone = Drone(drone_id=1, location=loc, max_capacity=2.0)
        self.assertEqual(drone.drone_id, 1)
        self.assertEqual(drone.location, loc)
        self.assertEqual(drone.max_capacity, 2.0)
        self.assertEqual(drone.current_load, 0.0)
    
    def test_available_capacity(self):
        """Test available capacity calculation."""
        drone = Drone(drone_id=1, location=Location(0, 0), max_capacity=2.0, current_load=0.5)
        self.assertEqual(drone.available_capacity(), 1.5)
    
    def test_can_handle_task(self):
        """Test task handling capability."""
        drone = Drone(drone_id=1, location=Location(0, 0), max_capacity=2.0, current_load=0.5)
        task = Task(task_id=1, location=Location(1, 1), load_requirement=1.0)
        self.assertTrue(drone.can_handle_task(task))
        
        heavy_task = Task(task_id=2, location=Location(1, 1), load_requirement=2.0)
        self.assertFalse(drone.can_handle_task(heavy_task))


class TestTask(unittest.TestCase):
    """Test Task class."""
    
    def test_task_creation(self):
        """Test creating a task."""
        loc = Location(5, 10)
        task = Task(task_id=1, location=loc, priority=2.0, load_requirement=0.5)
        self.assertEqual(task.task_id, 1)
        self.assertEqual(task.location, loc)
        self.assertEqual(task.priority, 2.0)
        self.assertEqual(task.load_requirement, 0.5)


class TestAssignmentResult(unittest.TestCase):
    """Test AssignmentResult class."""
    
    def test_get_summary(self):
        """Test getting summary of assignment result."""
        drone = Drone(drone_id=1, location=Location(0, 0))
        task = Task(task_id=1, location=Location(3, 4))
        assignment = Assignment(drone=drone, task=task, distance=5.0, cost=5.0)
        
        result = AssignmentResult(
            assignments=[assignment],
            total_cost=5.0,
            total_distance=5.0,
            unassigned_tasks=[],
            computation_time=0.1,
            strategy_name="Test Strategy"
        )
        
        summary = result.get_summary()
        self.assertEqual(summary["strategy"], "Test Strategy")
        self.assertEqual(summary["num_assignments"], 1)
        self.assertEqual(summary["num_unassigned"], 0)
        self.assertEqual(summary["total_cost"], 5.0)
        self.assertEqual(summary["average_distance"], 5.0)


if __name__ == '__main__':
    unittest.main()
