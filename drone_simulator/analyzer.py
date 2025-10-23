"""
Analyzer for comparing and visualizing assignment results.
"""

from typing import List, Dict, Any, Optional
from .models import AssignmentResult
import json


class AssignmentAnalyzer:
    """Analyzes and compares assignment results from different strategies."""
    
    def __init__(self):
        """Initialize the analyzer."""
        self.results: List[AssignmentResult] = []
    
    def add_result(self, result: AssignmentResult):
        """Add a result to analyze."""
        self.results.append(result)
    
    def clear_results(self):
        """Clear all stored results."""
        self.results = []
    
    def compare_strategies(self) -> Dict[str, Any]:
        """
        Compare all stored results.
        
        Returns:
            Dictionary with comparison metrics
        """
        if not self.results:
            return {}
        
        comparison = {
            "strategies": [],
            "best_cost": None,
            "best_distance": None,
            "best_assignments": None,
            "fastest": None,
        }
        
        best_cost_value = float('inf')
        best_distance_value = float('inf')
        best_assignments_count = -1
        fastest_time = float('inf')
        
        for result in self.results:
            summary = result.get_summary()
            comparison["strategies"].append(summary)
            
            # Track best metrics
            if result.total_cost < best_cost_value:
                best_cost_value = result.total_cost
                comparison["best_cost"] = result.strategy_name
            
            if result.total_distance < best_distance_value:
                best_distance_value = result.total_distance
                comparison["best_distance"] = result.strategy_name
            
            if len(result.assignments) > best_assignments_count:
                best_assignments_count = len(result.assignments)
                comparison["best_assignments"] = result.strategy_name
            
            if result.computation_time < fastest_time:
                fastest_time = result.computation_time
                comparison["fastest"] = result.strategy_name
        
        return comparison
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get statistical summary of all results.
        
        Returns:
            Dictionary with statistical metrics
        """
        if not self.results:
            return {}
        
        costs = [r.total_cost for r in self.results]
        distances = [r.total_distance for r in self.results]
        times = [r.computation_time for r in self.results]
        assignments = [len(r.assignments) for r in self.results]
        
        return {
            "num_strategies": len(self.results),
            "cost": {
                "min": min(costs),
                "max": max(costs),
                "avg": sum(costs) / len(costs),
            },
            "distance": {
                "min": min(distances),
                "max": max(distances),
                "avg": sum(distances) / len(distances),
            },
            "computation_time": {
                "min": min(times),
                "max": max(times),
                "avg": sum(times) / len(times),
            },
            "assignments": {
                "min": min(assignments),
                "max": max(assignments),
                "avg": sum(assignments) / len(assignments),
            },
        }
    
    def print_comparison(self):
        """Print a formatted comparison of all results."""
        if not self.results:
            print("No results to compare.")
            return
        
        print("=" * 80)
        print("ASSIGNMENT STRATEGY COMPARISON")
        print("=" * 80)
        
        for result in self.results:
            print(f"\nStrategy: {result.strategy_name}")
            print("-" * 80)
            print(f"  Assignments:       {len(result.assignments)}")
            print(f"  Unassigned Tasks:  {len(result.unassigned_tasks)}")
            print(f"  Total Cost:        {result.total_cost:.2f}")
            print(f"  Total Distance:    {result.total_distance:.2f}")
            if result.assignments:
                print(f"  Average Distance:  {result.total_distance / len(result.assignments):.2f}")
            print(f"  Computation Time:  {result.computation_time:.6f}s")
        
        print("\n" + "=" * 80)
        print("BEST PERFORMERS")
        print("=" * 80)
        
        comparison = self.compare_strategies()
        print(f"  Lowest Cost:       {comparison['best_cost']}")
        print(f"  Shortest Distance: {comparison['best_distance']}")
        print(f"  Most Assignments:  {comparison['best_assignments']}")
        print(f"  Fastest:           {comparison['fastest']}")
        print("=" * 80)
    
    def export_to_json(self, filepath: str):
        """
        Export results to JSON file.
        
        Args:
            filepath: Path to output JSON file
        """
        data = {
            "comparison": self.compare_strategies(),
            "statistics": self.get_statistics(),
            "results": [result.get_summary() for result in self.results]
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    def visualize(self, save_path: Optional[str] = None):
        """
        Create visualization of assignment results.
        Requires matplotlib library.
        
        Args:
            save_path: Path to save the figure (if None, displays interactively)
        """
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
        except ImportError:
            print("matplotlib is required for visualization. Install it with: pip install matplotlib")
            return
        
        if not self.results:
            print("No results to visualize.")
            return
        
        # Create subplots for each strategy
        num_results = len(self.results)
        fig, axes = plt.subplots(1, num_results, figsize=(6 * num_results, 6))
        
        if num_results == 1:
            axes = [axes]
        
        for idx, result in enumerate(self.results):
            ax = axes[idx]
            
            # Plot drones
            if result.assignments:
                drone_xs = [a.drone.location.x for a in result.assignments]
                drone_ys = [a.drone.location.y for a in result.assignments]
                ax.scatter(drone_xs, drone_ys, c='blue', marker='^', s=100, label='Drones', alpha=0.7)
                
                # Plot tasks
                task_xs = [a.task.location.x for a in result.assignments]
                task_ys = [a.task.location.y for a in result.assignments]
                ax.scatter(task_xs, task_ys, c='red', marker='o', s=100, label='Tasks', alpha=0.7)
                
                # Plot assignments as lines
                for assignment in result.assignments:
                    ax.plot(
                        [assignment.drone.location.x, assignment.task.location.x],
                        [assignment.drone.location.y, assignment.task.location.y],
                        'g--', alpha=0.3
                    )
            
            # Plot unassigned tasks
            if result.unassigned_tasks:
                unassigned_xs = [t.location.x for t in result.unassigned_tasks]
                unassigned_ys = [t.location.y for t in result.unassigned_tasks]
                ax.scatter(unassigned_xs, unassigned_ys, c='gray', marker='x', s=100, label='Unassigned', alpha=0.7)
            
            ax.set_title(f"{result.strategy_name}\nCost: {result.total_cost:.2f}")
            ax.set_xlabel("X coordinate")
            ax.set_ylabel("Y coordinate")
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Visualization saved to {save_path}")
        else:
            plt.show()
