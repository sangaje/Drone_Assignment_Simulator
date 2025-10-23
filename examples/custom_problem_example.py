"""
Example of creating a custom problem with user-defined assignment.
"""

from drone_simulator import (
    Drone,
    Task,
    Location,
    AssignmentProblem,
    UserDefinedAssignment,
    NearestAssignment,
    AssignmentAnalyzer,
)


def main():
    print("=" * 80)
    print("Drone Assignment Simulator - Custom Problem Example")
    print("=" * 80)
    
    # Create a custom problem
    print("\nCreating custom problem...")
    problem = AssignmentProblem()
    
    # Add drones at specific locations
    drones_data = [
        (0, 0, 2.0),    # Drone 0: bottom-left, capacity 2.0
        (100, 0, 2.0),  # Drone 1: bottom-right, capacity 2.0
        (0, 100, 2.0),  # Drone 2: top-left, capacity 2.0
        (100, 100, 2.0) # Drone 3: top-right, capacity 2.0
    ]
    
    for i, (x, y, capacity) in enumerate(drones_data):
        drone = Drone(
            drone_id=i,
            location=Location(x, y),
            max_capacity=capacity
        )
        problem.add_drone(drone)
        print(f"  Added Drone {i} at ({x}, {y})")
    
    # Add tasks at specific locations
    tasks_data = [
        (10, 10, 1.5, 1.0),   # Task 0: near drone 0, high priority
        (90, 10, 1.0, 0.5),   # Task 1: near drone 1
        (10, 90, 2.0, 1.0),   # Task 2: near drone 2, highest priority
        (90, 90, 1.2, 0.5),   # Task 3: near drone 3
        (50, 50, 1.8, 1.5),   # Task 4: center, high load
    ]
    
    for i, (x, y, priority, load) in enumerate(tasks_data):
        task = Task(
            task_id=i,
            location=Location(x, y),
            priority=priority,
            load_requirement=load
        )
        problem.add_task(task)
        print(f"  Added Task {i} at ({x}, {y}), priority={priority}, load={load}")
    
    print(f"\nCreated custom problem with {len(problem.drones)} drones and {len(problem.tasks)} tasks")
    
    # Create analyzer
    analyzer = AssignmentAnalyzer()
    
    # Solve with user-defined assignment
    print("\n" + "-" * 80)
    print("Solving with user-defined assignment...")
    
    # Define custom assignment: assign each task to its nearest corner drone
    user_mapping = {
        0: 0,  # Task 0 (bottom-left area) -> Drone 0
        1: 1,  # Task 1 (bottom-right area) -> Drone 1
        2: 2,  # Task 2 (top-left area) -> Drone 2
        3: 3,  # Task 3 (top-right area) -> Drone 3
        4: 0,  # Task 4 (center) -> Drone 0
    }
    
    print("User mapping:")
    for task_id, drone_id in user_mapping.items():
        print(f"  Task {task_id} -> Drone {drone_id}")
    
    user_strategy = UserDefinedAssignment(user_mapping)
    user_result = user_strategy.solve(problem)
    analyzer.add_result(user_result)
    
    print(f"\nResult:")
    print(f"  Assignments: {len(user_result.assignments)}")
    print(f"  Unassigned: {len(user_result.unassigned_tasks)}")
    print(f"  Total Cost: {user_result.total_cost:.2f}")
    print(f"  Total Distance: {user_result.total_distance:.2f}")
    
    # Compare with automatic assignment
    print("\n" + "-" * 80)
    print("Solving with nearest assignment (automatic)...")
    
    nearest_strategy = NearestAssignment(consider_priority=True)
    nearest_result = nearest_strategy.solve(problem)
    analyzer.add_result(nearest_result)
    
    print(f"Result:")
    print(f"  Assignments: {len(nearest_result.assignments)}")
    print(f"  Unassigned: {len(nearest_result.unassigned_tasks)}")
    print(f"  Total Cost: {nearest_result.total_cost:.2f}")
    print(f"  Total Distance: {nearest_result.total_distance:.2f}")
    
    # Print comparison
    print("\n")
    analyzer.print_comparison()
    
    # Show detailed assignments
    print("\n" + "=" * 80)
    print("DETAILED ASSIGNMENTS")
    print("=" * 80)
    
    for result in analyzer.results:
        print(f"\n{result.strategy_name}:")
        print("-" * 80)
        for assignment in result.assignments:
            print(f"  Drone {assignment.drone.drone_id} -> Task {assignment.task.task_id}: "
                  f"distance={assignment.distance:.2f}, cost={assignment.cost:.2f}")
        if result.unassigned_tasks:
            print(f"  Unassigned: {[t.task_id for t in result.unassigned_tasks]}")
    
    # Try to visualize
    try:
        print("\n" + "=" * 80)
        print("Generating visualization...")
        analyzer.visualize(save_path="custom_problem.png")
        print("Visualization saved to custom_problem.png")
    except ImportError:
        print("\nNote: Install matplotlib to enable visualization: pip install matplotlib")
    
    print("\n" + "=" * 80)
    print("Custom problem example completed!")
    print("=" * 80)


if __name__ == "__main__":
    main()
