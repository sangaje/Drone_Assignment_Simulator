# Drone Assignment Simulator

A Python tool for testing and analyzing drone assignment problems with multiple strategies.

## Features

- **Multiple Assignment Strategies**:
  - **Random Assignment**: Randomly assigns tasks to available drones
  - **Nearest Assignment**: Greedy algorithm that assigns each task to the nearest available drone
  - **MILP Assignment**: Optimal assignment using Mixed Integer Linear Programming
  - **User-Defined Assignment**: Allows custom assignment mappings

- **Flexible Problem Definition**:
  - Generate random problems with configurable parameters
  - Define custom problems with specific drones and tasks
  - Support for drone capacity constraints and task priorities

- **Analysis and Comparison**:
  - Compare multiple strategies on the same problem
  - Statistical analysis of results
  - Visualization of assignments (with matplotlib)
  - Export results to JSON

## Installation

1. Clone the repository:
```bash
git clone https://github.com/sangaje/Drone_Assignment_Simulator.git
cd Drone_Assignment_Simulator
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

Optional dependencies:
- For MILP optimization: `pip install pulp`
- For visualization: `pip install matplotlib`

## Quick Start

```python
from drone_simulator import (
    AssignmentProblem,
    RandomAssignment,
    NearestAssignment,
    AssignmentAnalyzer
)

# Generate a random problem
problem = AssignmentProblem.generate_random_problem(
    num_drones=5,
    num_tasks=10,
    area_size=100.0,
    seed=42
)

# Solve with different strategies
random_strategy = RandomAssignment(seed=42)
random_result = random_strategy.solve(problem)

nearest_strategy = NearestAssignment(consider_priority=True)
nearest_result = nearest_strategy.solve(problem)

# Analyze and compare results
analyzer = AssignmentAnalyzer()
analyzer.add_result(random_result)
analyzer.add_result(nearest_result)
analyzer.print_comparison()
```

## Usage Examples

### Creating a Custom Problem

```python
from drone_simulator import Drone, Task, Location, AssignmentProblem

# Create drones
drone1 = Drone(drone_id=0, location=Location(0, 0), max_capacity=2.0)
drone2 = Drone(drone_id=1, location=Location(50, 50), max_capacity=2.0)

# Create tasks
task1 = Task(task_id=0, location=Location(10, 10), priority=1.5, load_requirement=1.0)
task2 = Task(task_id=1, location=Location(40, 40), priority=2.0, load_requirement=0.5)

# Create problem
problem = AssignmentProblem(drones=[drone1, drone2], tasks=[task1, task2])
```

### Using MILP Optimal Assignment

```python
from drone_simulator import MILPAssignment

# Requires pulp: pip install pulp
milp_strategy = MILPAssignment(time_limit=60)
result = milp_strategy.solve(problem)

print(f"Optimal cost: {result.total_cost}")
print(f"Assignments: {len(result.assignments)}")
```

### User-Defined Assignment

```python
from drone_simulator import UserDefinedAssignment

# Define custom mapping: task_id -> drone_id
assignment_map = {
    0: 1,  # Assign task 0 to drone 1
    1: 0,  # Assign task 1 to drone 0
}

user_strategy = UserDefinedAssignment(assignment_map)
result = user_strategy.solve(problem)
```

### Visualization

```python
# Visualize results (requires matplotlib)
analyzer.visualize(save_path='assignments.png')
```

### Export Results

```python
# Export comparison to JSON
analyzer.export_to_json('results.json')
```

## Running Tests

```bash
python -m pytest tests/
```

Or using unittest:

```bash
python -m unittest discover tests
```

## Project Structure

```
Drone_Assignment_Simulator/
├── drone_simulator/
│   ├── __init__.py
│   ├── models.py          # Core data models
│   ├── problem.py         # Problem definition
│   ├── strategies.py      # Assignment strategies
│   └── analyzer.py        # Analysis tools
├── tests/
│   ├── test_models.py
│   ├── test_strategies.py
│   └── test_analyzer.py
├── examples/
│   ├── basic_example.py
│   ├── comparison_example.py
│   └── milp_example.py
└── README.md
```

## Requirements

- Python 3.8+
- Optional: pulp (for MILP optimization)
- Optional: matplotlib (for visualization)

## License

See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.