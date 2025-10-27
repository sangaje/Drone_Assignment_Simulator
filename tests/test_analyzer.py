"""
Tests for assignment analyzer.
"""

import unittest
import os
import tempfile
from drone_simulator.problem import AssignmentProblem
from drone_simulator.strategies import RandomAssignment, NearestAssignment
from drone_simulator.analyzer import AssignmentAnalyzer


class TestAssignmentAnalyzer(unittest.TestCase):
    """Test AssignmentAnalyzer class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.analyzer = AssignmentAnalyzer()
        
        # Create a test problem
        self.problem = AssignmentProblem.generate_random_problem(
            num_drones=3,
            num_tasks=5,
            seed=42
        )
    
    def test_add_result(self):
        """Test adding results to analyzer."""
        strategy = RandomAssignment(seed=42)
        result = strategy.solve(self.problem)
        
        self.analyzer.add_result(result)
        self.assertEqual(len(self.analyzer.results), 1)
    
    def test_compare_strategies(self):
        """Test comparing multiple strategies."""
        # Solve with multiple strategies
        random_strategy = RandomAssignment(seed=42)
        random_result = random_strategy.solve(self.problem)
        self.analyzer.add_result(random_result)
        
        nearest_strategy = NearestAssignment()
        nearest_result = nearest_strategy.solve(self.problem)
        self.analyzer.add_result(nearest_result)
        
        comparison = self.analyzer.compare_strategies()
        
        self.assertIn("strategies", comparison)
        self.assertEqual(len(comparison["strategies"]), 2)
        self.assertIsNotNone(comparison["best_cost"])
        self.assertIsNotNone(comparison["fastest"])
    
    def test_get_statistics(self):
        """Test getting statistics."""
        random_strategy = RandomAssignment(seed=42)
        random_result = random_strategy.solve(self.problem)
        self.analyzer.add_result(random_result)
        
        nearest_strategy = NearestAssignment()
        nearest_result = nearest_strategy.solve(self.problem)
        self.analyzer.add_result(nearest_result)
        
        stats = self.analyzer.get_statistics()
        
        self.assertIn("num_strategies", stats)
        self.assertEqual(stats["num_strategies"], 2)
        self.assertIn("cost", stats)
        self.assertIn("distance", stats)
        self.assertIn("computation_time", stats)
    
    def test_export_to_json(self):
        """Test exporting results to JSON."""
        random_strategy = RandomAssignment(seed=42)
        random_result = random_strategy.solve(self.problem)
        self.analyzer.add_result(random_result)
        
        # Create temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            temp_path = f.name
        
        try:
            self.analyzer.export_to_json(temp_path)
            self.assertTrue(os.path.exists(temp_path))
            
            # Verify file is valid JSON
            import json
            with open(temp_path, 'r') as f:
                data = json.load(f)
                self.assertIn("comparison", data)
                self.assertIn("statistics", data)
                self.assertIn("results", data)
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)
    
    def test_clear_results(self):
        """Test clearing results."""
        random_strategy = RandomAssignment(seed=42)
        random_result = random_strategy.solve(self.problem)
        self.analyzer.add_result(random_result)
        
        self.assertEqual(len(self.analyzer.results), 1)
        
        self.analyzer.clear_results()
        self.assertEqual(len(self.analyzer.results), 0)


if __name__ == '__main__':
    unittest.main()
