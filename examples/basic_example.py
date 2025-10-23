"""
Basic example of using the drone assignment simulator.
"""

from drone_simulator import (
    AssignmentProblem,
    RandomAssignment,
    NearestAssignment,
)


def main():
    print("=" * 80)
    print("Drone Assignment Simulator - Basic Example")
    print("=" * 80)
    
    # Generate a random problem
    print("\nGenerating random problem...")
    problem = AssignmentProblem.generate_random_problem(
        num_drones=5,
        num_tasks=10,
        area_size=100.0,
        seed=42
    )
    print(f"Created problem with {len(problem.drones)} drones and {len(problem.tasks)} tasks")
    
    # Solve with Random Assignment
    print("\n" + "-" * 80)
    print("Solving with Random Assignment...")
    random_strategy = RandomAssignment(seed=42)
    random_result = random_strategy.solve(problem)
    print(f"Strategy: {random_result.strategy_name}")
    print(f"Assignments: {len(random_result.assignments)}")
    print(f"Unassigned Tasks: {len(random_result.unassigned_tasks)}")
    print(f"Total Cost: {random_result.total_cost:.2f}")
    print(f"Total Distance: {random_result.total_distance:.2f}")
    print(f"Computation Time: {random_result.computation_time:.6f}s")
    
    # Solve with Nearest Assignment
    print("\n" + "-" * 80)
    print("Solving with Nearest Assignment...")
    nearest_strategy = NearestAssignment(consider_priority=True)
    nearest_result = nearest_strategy.solve(problem)
    print(f"Strategy: {nearest_result.strategy_name}")
    print(f"Assignments: {len(nearest_result.assignments)}")
    print(f"Unassigned Tasks: {len(nearest_result.unassigned_tasks)}")
    print(f"Total Cost: {nearest_result.total_cost:.2f}")
    print(f"Total Distance: {nearest_result.total_distance:.2f}")
    print(f"Computation Time: {nearest_result.computation_time:.6f}s")
    
    print("\n" + "=" * 80)
    print("Example completed!")
    print("=" * 80)


if __name__ == "__main__":
    main()
