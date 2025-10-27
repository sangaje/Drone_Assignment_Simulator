"""
Example comparing multiple assignment strategies.
"""

from drone_simulator import (
    AssignmentProblem,
    RandomAssignment,
    NearestAssignment,
    AssignmentAnalyzer,
)


def main():
    print("=" * 80)
    print("Drone Assignment Simulator - Strategy Comparison")
    print("=" * 80)
    
    # Generate a random problem
    print("\nGenerating random problem...")
    problem = AssignmentProblem.generate_random_problem(
        num_drones=8,
        num_tasks=20,
        area_size=100.0,
        seed=42
    )
    print(f"Created problem with {len(problem.drones)} drones and {len(problem.tasks)} tasks")
    
    # Create analyzer
    analyzer = AssignmentAnalyzer()
    
    # Solve with multiple strategies
    strategies = [
        RandomAssignment(seed=42),
        RandomAssignment(seed=123),
        NearestAssignment(consider_priority=False),
        NearestAssignment(consider_priority=True),
    ]
    
    print("\nSolving with different strategies...")
    for strategy in strategies:
        result = strategy.solve(problem)
        analyzer.add_result(result)
        print(f"  ✓ {strategy.get_name()}")
    
    # Print comparison
    print("\n")
    analyzer.print_comparison()
    
    # Get statistics
    print("\n" + "=" * 80)
    print("STATISTICS")
    print("=" * 80)
    stats = analyzer.get_statistics()
    
    print(f"\nCost Statistics:")
    print(f"  Min:  {stats['cost']['min']:.2f}")
    print(f"  Max:  {stats['cost']['max']:.2f}")
    print(f"  Avg:  {stats['cost']['avg']:.2f}")
    
    print(f"\nDistance Statistics:")
    print(f"  Min:  {stats['distance']['min']:.2f}")
    print(f"  Max:  {stats['distance']['max']:.2f}")
    print(f"  Avg:  {stats['distance']['avg']:.2f}")
    
    print(f"\nComputation Time Statistics:")
    print(f"  Min:  {stats['computation_time']['min']:.6f}s")
    print(f"  Max:  {stats['computation_time']['max']:.6f}s")
    print(f"  Avg:  {stats['computation_time']['avg']:.6f}s")
    
    # Export results
    output_file = "comparison_results.json"
    analyzer.export_to_json(output_file)
    print(f"\nResults exported to {output_file}")
    
    # Try to visualize (if matplotlib is available)
    try:
        print("\nGenerating visualization...")
        analyzer.visualize(save_path="comparison_plot.png")
        print("Visualization saved to comparison_plot.png")
    except ImportError:
        print("\nNote: Install matplotlib to enable visualization: pip install matplotlib")
    
    print("\n" + "=" * 80)
    print("Comparison completed!")
    print("=" * 80)


if __name__ == "__main__":
    main()
