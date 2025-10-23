"""
Example using MILP optimization for optimal assignment.
"""

from drone_simulator import (
    AssignmentProblem,
    MILPAssignment,
    NearestAssignment,
    AssignmentAnalyzer,
)


def main():
    print("=" * 80)
    print("Drone Assignment Simulator - MILP Optimization Example")
    print("=" * 80)
    
    # Check if pulp is available
    try:
        import pulp
        print("\n✓ PuLP library is available")
    except ImportError:
        print("\n✗ PuLP library is not installed")
        print("Install it with: pip install pulp")
        return
    
    # Generate a smaller problem (MILP can be slow for large problems)
    print("\nGenerating random problem...")
    problem = AssignmentProblem.generate_random_problem(
        num_drones=5,
        num_tasks=10,
        area_size=100.0,
        seed=42
    )
    print(f"Created problem with {len(problem.drones)} drones and {len(problem.tasks)} tasks")
    
    # Create analyzer
    analyzer = AssignmentAnalyzer()
    
    # Solve with MILP (optimal)
    print("\n" + "-" * 80)
    print("Solving with MILP (optimal solution)...")
    milp_strategy = MILPAssignment(time_limit=60)
    milp_result = milp_strategy.solve(problem)
    analyzer.add_result(milp_result)
    
    print(f"Strategy: {milp_result.strategy_name}")
    print(f"Solver Status: {milp_result.metadata.get('solver_status', 'N/A')}")
    print(f"Assignments: {len(milp_result.assignments)}")
    print(f"Unassigned Tasks: {len(milp_result.unassigned_tasks)}")
    print(f"Total Cost: {milp_result.total_cost:.2f}")
    print(f"Total Distance: {milp_result.total_distance:.2f}")
    print(f"Computation Time: {milp_result.computation_time:.6f}s")
    
    # Compare with greedy approach
    print("\n" + "-" * 80)
    print("Solving with Nearest Assignment (greedy)...")
    nearest_strategy = NearestAssignment(consider_priority=True)
    nearest_result = nearest_strategy.solve(problem)
    analyzer.add_result(nearest_result)
    
    print(f"Strategy: {nearest_result.strategy_name}")
    print(f"Assignments: {len(nearest_result.assignments)}")
    print(f"Unassigned Tasks: {len(nearest_result.unassigned_tasks)}")
    print(f"Total Cost: {nearest_result.total_cost:.2f}")
    print(f"Total Distance: {nearest_result.total_distance:.2f}")
    print(f"Computation Time: {nearest_result.computation_time:.6f}s")
    
    # Print comparison
    print("\n")
    analyzer.print_comparison()
    
    # Calculate improvement
    if nearest_result.total_cost > 0:
        improvement = ((nearest_result.total_cost - milp_result.total_cost) / 
                      nearest_result.total_cost * 100)
        print(f"\nMILP improvement over greedy: {improvement:.2f}%")
    
    # Try to visualize
    try:
        print("\nGenerating visualization...")
        analyzer.visualize(save_path="milp_comparison.png")
        print("Visualization saved to milp_comparison.png")
    except ImportError:
        print("\nNote: Install matplotlib to enable visualization: pip install matplotlib")
    
    print("\n" + "=" * 80)
    print("MILP example completed!")
    print("=" * 80)


if __name__ == "__main__":
    main()
