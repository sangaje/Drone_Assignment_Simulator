"""Simplified MILP for Drone→Task Assignment using SciPy HiGHS.

Objective (Min):
  sum_{i,j} (α·d̄_{ij} + β·ē_{ij} − γ·Ā_j)·x_{ij}  +  sum_j ρ_j·y_j

Constraints:
 (1) ∀j: sum_i x_{ij} + y_j = 1    - Each task assigned to one drone or dropped
 (2) ∀i: sum_j x_{ij} ≤ Q_i        - Drone capacity limit
 (3) ∀i: sum_j e_{ij}·x_{ij} ≤ E_i - Battery constraint
 (4) ∀i,j: x_{ij} ≤ w_{ij}         - Distance gate constraint
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp

# ============================================================================
# CONFIGURATION
# ============================================================================

def get_default_params():
    """Get default MILP optimization parameters."""
    return {
        'alpha': 1.0,           # Distance weight
        'beta': 1.0,            # Energy weight
        'gamma': 1.0,           # Priority weight
        'sigma': 0.2,           # Safety margin (20% battery reserve)
        'time_limit': 5.0,      # Solver time limit (seconds)
        'mip_rel_gap': 1e-3     # Relative optimality gap
    }


# ============================================================================
# DATA PREPARATION
# ============================================================================

def create_sample_data():
    """Create sample problem data for testing."""
    n_drones, n_tasks = 3, 6

    # Round-trip flight distance [km]
    d = np.array([
        [ 6,  8, 12,  5,  9,  7],
        [ 4, 10,  3,  6, 11,  8],
        [ 9,  6,  7, 12, 13,  5],
    ], dtype=float)

    # Energy model: e = κ·d + e_hover [Wh]
    kappa = np.array([12.0, 14.0, 11.0])  # Wh/km
    e_hover = np.array([10, 12, 8, 9, 11, 7], float)
    e = kappa[:, None] * d + e_hover[None, :]

    # Drone battery capacity [Wh]
    battery_capacity = np.array([200.0, 220.0, 180.0])

    # Task priority
    priority = np.array([0.9, 0.2, 0.4, 0.7, 0.3, 0.6])

    # Distance gate: w ∈ {0,1}
    max_distance = np.array([7.0, 8.0, 6.0])  # km (one-way)
    w = ((d/2.0) <= max_distance[:, None]).astype(float)

    # Capacity per drone
    capacity = np.ones(n_drones, dtype=int)

    # Drop penalty
    rho = 0.6 * np.ones(n_tasks, float)

    return {
        'n_drones': n_drones,
        'n_tasks': n_tasks,
        'd': d,
        'e': e,
        'battery_capacity': battery_capacity,
        'priority': priority,
        'w': w,
        'capacity': capacity,
        'rho': rho
    }


# ============================================================================
# MILP PROBLEM SETUP
# ============================================================================

def build_milp_problem(data, params):
    """Build complete MILP problem: objective, bounds, and constraints."""
    n_drones = data['n_drones']
    n_tasks = data['n_tasks']
    n_x = n_drones * n_tasks  # x_{ij} variables
    n_y = n_tasks             # y_j variables
    n_total = n_x + n_y

    # Helper functions for variable indexing
    def idx_x(i, j): return i * n_tasks + j
    def idx_y(j): return n_x + j

    # Normalize parameters for numerical stability
    d_max = max(np.max(data['d']), 1.0)
    a_max = max(np.max(data['priority']), 1.0)
    battery_safe = np.where(data['battery_capacity'] > 0, data['battery_capacity'], 1.0)

    d_bar = data['d'] / d_max
    e_bar = data['e'] / battery_safe[:, None]
    a_bar = data['priority'] / a_max

    # Build objective function: c^T x
    c_x = (params['alpha'] * d_bar +
           params['beta'] * e_bar -
           params['gamma'] * a_bar[None, :])
    c = np.zeros(n_total, float)
    c[:n_x] = c_x.ravel(order="C")
    c[n_x:] = data['rho']

    # Build variable bounds
    lb = np.zeros(n_total)
    ub = np.ones(n_total)
    ub_x = ub[:n_x].reshape(n_drones, n_tasks)
    ub_x[:] = data['w']  # Apply distance gate

    # Mark energy-infeasible pairs
    battery_limit = (1.0 - params['sigma']) * data['battery_capacity']
    energy_infeasible = data['e'] > (battery_limit[:, None] + 1e-12)
    ub_x[energy_infeasible] = 0.0

    ub[:n_x] = ub_x.ravel(order="C")
    bounds = Bounds(lb, ub)

    # Build constraints
    constraints = []

    # (1) Assignment: each task assigned to one drone or dropped
    A_eq = np.zeros((n_tasks, n_total))
    for j in range(n_tasks):
        for i in range(n_drones):
            A_eq[j, idx_x(i, j)] = 1.0
        A_eq[j, idx_y(j)] = 1.0
    constraints.append(LinearConstraint(A_eq, lb=np.ones(n_tasks), ub=np.ones(n_tasks)))

    # (2) Capacity: each drone task limit
    A_cap = np.zeros((n_drones, n_total))
    for i in range(n_drones):
        for j in range(n_tasks):
            A_cap[i, idx_x(i, j)] = 1.0
    constraints.append(LinearConstraint(A_cap, lb=-np.inf*np.ones(n_drones),
                                       ub=data['capacity'].astype(float)))

    # (3) Battery: energy consumption limit
    A_battery = np.zeros((n_drones, n_total))
    for i in range(n_drones):
        for j in range(n_tasks):
            if ub_x[i, j] > 0.0:
                A_battery[i, idx_x(i, j)] = data['e'][i, j]
    constraints.append(LinearConstraint(A_battery, lb=-np.inf*np.ones(n_drones),
                                       ub=battery_limit.astype(float)))

    return c, bounds, constraints, ub_x, battery_limit


# ============================================================================
# SOLVE & PARSE RESULTS
# ============================================================================

def solve_and_parse(c, bounds, constraints, data, params):
    """Solve MILP and parse results."""
    n_drones = data['n_drones']
    n_tasks = data['n_tasks']
    n_x = n_drones * n_tasks
    n_total = len(c)

    # Solve
    integrality = np.ones(n_total, dtype=int)
    res = milp(
        c=c,
        integrality=integrality,
        bounds=bounds,
        constraints=constraints,
        options={
            "time_limit": params['time_limit'],
            "mip_rel_gap": params['mip_rel_gap'],
            "presolve": True
        }
    )

    # Parse solution
    x_bin = np.zeros((n_drones, n_tasks), dtype=int)
    y_bin = np.zeros(n_tasks, dtype=int)

    if res.x is not None:
        x_raw = res.x[:n_x].reshape(n_drones, n_tasks)
        y_raw = res.x[n_x:]
        x_bin = (x_raw > 0.5).astype(int)
        y_bin = (y_raw > 0.5).astype(int)

    assigned = [(i, j) for i in range(n_drones) for j in range(n_tasks) if x_bin[i, j] == 1]
    dropped = [j for j in range(n_tasks) if y_bin[j] == 1]
    objective = float(res.fun) if res.fun is not None else float('inf')

    return {
        'x_bin': x_bin,
        'y_bin': y_bin,
        'assigned': assigned,
        'dropped': dropped,
        'objective': objective,
        'status': res.status,
        'message': res.message
    }


# ============================================================================
# VALIDATION & DISPLAY
# ============================================================================

def validate_solution(result, data, battery_limit):
    """Validate solution satisfies all constraints."""
    x_bin, y_bin = result['x_bin'], result['y_bin']

    assert np.all(x_bin.sum(axis=0) + y_bin == 1), "Assignment constraint violated"
    assert np.all(x_bin.sum(axis=1) <= data['capacity']), "Capacity constraint violated"
    assert np.all((data['e'] * x_bin).sum(axis=1) <= battery_limit + 1e-6), "Battery constraint violated"
    assert np.all(x_bin <= data['w'] + 1e-9), "Distance gate constraint violated"


def print_results(result, data, params, ub_x):
    """Print solution results and analysis."""
    print("=== Solver Status ===")
    print(f"Status: {result['status']} - {result['message']}")
    print(f"Objective: {result['objective']:.6f}\n")

    print("=== Assignment Results ===")
    print(f"Assigned (drone→task): {result['assigned']}")
    print(f"Dropped tasks: {result['dropped']}")

    # Per-task analysis
    print("\n=== Per-Task Analysis ===")
    n_drones = data['n_drones']
    n_tasks = data['n_tasks']
    MASK = (ub_x <= 0.0)

    d_max = max(np.max(data['d']), 1.0)
    a_max = max(np.max(data['priority']), 1.0)

    c_full = (params['alpha'] * (data['d'] / d_max) +
              params['beta'] * (data['e'] / data['battery_capacity'][:, None]) -
              params['gamma'] * (data['priority'] / a_max)[None, :])
    c_masked = np.where(MASK, np.inf, c_full)

    for j in range(n_tasks):
        feasible = [i for i in range(n_drones) if not MASK[i, j]]
        min_cost = float(np.min(c_masked[:, j])) if feasible else float('inf')

        if result['y_bin'][j] == 0:
            decision = "ASSIGNED"
        elif not feasible:
            decision = "DROP(infeasible)"
        else:
            decision = "DROP(costly)"

        print(f"Task {j}: min_cost={min_cost:7.4f}, rho={data['rho'][j]:.2f}, "
              f"{decision}, feasible_drones={feasible}")


# ============================================================================
# MAIN
# ============================================================================

def main():
    """Run MILP drone-task assignment optimization."""
    np.set_printoptions(precision=3, suppress=True)

    # Setup
    params = get_default_params()
    data = create_sample_data()

    # Build and solve
    c, bounds, constraints, ub_x, battery_limit = build_milp_problem(data, params)
    result = solve_and_parse(c, bounds, constraints, data, params)

    # Validate and display
    validate_solution(result, data, battery_limit)
    print_results(result, data, params, ub_x)


if __name__ == "__main__":
    main()
