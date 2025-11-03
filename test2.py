from __future__ import annotations

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp

# MILP Optimization Hyperparameters
# Objective Function Weight Coefficients
ALPHA, BETA, GAMMA = 1.0, 1.0, 1.0  # Distance, Energy, Priority weights

# Constraint Parameters
SIGMA = 0.2  # Safety margin ratio (reserves 20% of battery capacity)

# Solver Parameters
TIME_LIMIT = 5.0      # Solver time limit in seconds
MIP_REL_GAP = 1e-3    # Relative optimality gap

# Drop Penalty Configuration
# Option 1: Uniform penalty for all tasks
RHO = 2.0 * (ALPHA + BETA + GAMMA)  # Balanced approach

def build_milp_problem(n_drone, n_task, d_bar, e_bar, a_bar, e, w, Q, E, rho = None):
    """Build complete MILP problem: objective, bounds, and constraints."""
    n_drones = n_drone
    n_tasks = n_task
    n_x = n_drones * n_tasks  # x_{ij} variables
    n_y = n_tasks             # y_j variables
    n_total = n_x + n_y

    # Helper functions for variable indexing
    def idx_x(i, j): return i * n_tasks + j
    def idx_y(j): return n_x + j

    # Normalize parameters for numerical stability
    # d_max = max(np.max(data['d']), 1.0)
    # a_max = max(np.max(data['priority']), 1.0)
    # battery_safe = np.where(data['battery_capacity'] > 0, data['battery_capacity'], 1.0)

    # Build objective function: c^T x
    c_x = (ALPHA * d_bar +
           BETA * e_bar -
           GAMMA * a_bar[None, :])

    if rho is None:
        rho = RHO * np.ones(n_tasks, float)

    c = np.zeros(n_total, float)
    c[:n_x] = c_x.ravel(order="C")
    c[n_x:] = rho

    # Build variable bounds
    lb = np.zeros(n_total)
    ub = np.ones(n_total)
    ub_x = ub[:n_x].reshape(n_drones, n_tasks)
    ub_x[:] = w  # Apply distance gate

    # Mark energy-infeasible pairs
    battery_limit = (1.0 - SIGMA) * E
    energy_infeasible = e > (battery_limit[:, None] + 1e-12)
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
                                       ub=Q.astype(float)))

    # (3) Battery: energy consumption limit
    A_battery = np.zeros((n_drones, n_total))
    for i in range(n_drones):
        for j in range(n_tasks):
            if ub_x[i, j] > 0.0:
                A_battery[i, idx_x(i, j)] = e[i, j]
    constraints.append(LinearConstraint(A_battery, lb=-np.inf*np.ones(n_drones),
                                       ub=battery_limit.astype(float)))

    return c, bounds, constraints, ub_x, battery_limit

def solve_and_parse(c, bounds, constraints, n_drones, n_tasks):
    """Solve MILP and parse results."""
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
            "time_limit": TIME_LIMIT,
            "mip_rel_gap": MIP_REL_GAP,
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

def validate_solution(result, data, battery_limit):
    """Validate solution satisfies all constraints."""
    x_bin, y_bin = result['x_bin'], result['y_bin']

    assert np.all(x_bin.sum(axis=0) + y_bin == 1), "Assignment constraint violated"
    assert np.all(x_bin.sum(axis=1) <= data['capacity']), "Capacity constraint violated"
    assert np.all((data['e'] * x_bin).sum(axis=1) <= battery_limit + 1e-6), "Battery constraint violated"
    assert np.all(x_bin <= data['w'] + 1e-9), "Distance gate constraint violated"


# ============================================================================
# TEST CODE
# ============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("MILP Drone Assignment Test")
    print("=" * 70)

    # Hyperparameters
    ALPHA, BETA, GAMMA = 1.0, 1.0, 1.0  # Weights
    SIGMA = 0.2  # Safety margin
    RHO = 2.0 * (ALPHA + BETA + GAMMA)  # Drop penalty
    TIME_LIMIT = 5.0
    MIP_REL_GAP = 1e-3

    # Test problem setup
    n_drones = 3
    n_tasks = 5

    print("\n📋 Problem Size:")
    print(f"   Drones: {n_drones}")
    print(f"   Tasks: {n_tasks}")
    print(f"   Total variables: {n_drones * n_tasks + n_tasks} (x: {n_drones * n_tasks}, y: {n_tasks})")

    # Generate test data
    np.random.seed(42)

    # Distance matrix (normalized): 드론-task 간 거리
    d_bar = np.random.rand(n_drones, n_tasks) * 0.8 + 0.2  # [0.2, 1.0]
    print("\n📏 Distance matrix (normalized):")
    print(d_bar)

    # Energy consumption matrix (normalized)
    e_bar = np.random.rand(n_drones, n_tasks) * 0.7 + 0.3  # [0.3, 1.0]
    print("\n🔋 Energy consumption (normalized):")
    print(e_bar)

    # Task priority (normalized)
    a_bar = np.random.rand(n_tasks) * 0.5 + 0.5  # [0.5, 1.0]
    print("\n⭐ Task priorities (normalized):")
    print(a_bar)

    # Raw energy consumption (Wh)
    e = np.random.rand(n_drones, n_tasks) * 3000 + 1000  # [1000, 4000] Wh
    print("\n⚡ Raw energy consumption (Wh):")
    print(e.astype(int))

    # Distance gate (binary): 범위 내 여부
    w = np.random.rand(n_drones, n_tasks) > 0.3  # 70% feasible
    w = w.astype(float)
    print("\n🚪 Distance gate (range feasibility):")
    print(w.astype(int))

    # Drone capacity (max concurrent tasks)
    Q = np.array([2, 1, 2])  # Heterogeneous fleet
    print("\n📦 Drone capacities (max tasks):")
    print(f"   Drone 0: {Q[0]} tasks")
    print(f"   Drone 1: {Q[1]} task")
    print(f"   Drone 2: {Q[2]} tasks")

    # Battery capacity (Wh)
    E = np.array([5000, 4000, 6000])  # Heterogeneous batteries
    print("\n🔋 Battery capacities (Wh):")
    for i, capacity in enumerate(E):
        print(f"   Drone {i}: {capacity} Wh (usable: {int((1-SIGMA)*capacity)} Wh)")

    # Build MILP problem
    print("\n" + "=" * 70)
    print("Building MILP problem...")
    print("=" * 70)

    c, bounds, constraints, ub_x, battery_limit = build_milp_problem(
        n_drone=n_drones,
        n_task=n_tasks,
        d_bar=d_bar,
        e_bar=e_bar,
        a_bar=a_bar,
        e=e,
        w=w,
        Q=Q,
        E=E,
        rho=None  # Use default
    )

    print("\n✅ Problem built successfully!")
    print(f"   Objective coefficients: {len(c)}")
    print(f"   Constraints: {len(constraints)}")
    print(f"   - Assignment (equality): {constraints[0].A.shape[0]} constraints")
    print(f"   - Capacity (inequality): {constraints[1].A.shape[0]} constraints")
    print(f"   - Battery (inequality): {constraints[2].A.shape[0]} constraints")

    # Show bounds summary
    print("\n🔢 Variable bounds summary:")
    ub_flat = bounds.ub
    n_blocked = np.sum(ub_flat[:n_drones*n_tasks] == 0)
    n_available = np.sum(ub_flat[:n_drones*n_tasks] == 1)
    print(f"   Blocked assignments (ub=0): {n_blocked}/{n_drones*n_tasks}")
    print(f"   Available assignments (ub=1): {n_available}/{n_drones*n_tasks}")
    print(f"   Feasibility rate: {n_available/(n_drones*n_tasks)*100:.1f}%")

    # Solve MILP
    print("\n" + "=" * 70)
    print("Solving MILP...")
    print("=" * 70)

    result = solve_and_parse(c, bounds, constraints, n_drones, n_tasks)

    # Display results
    print("\n📊 Optimization Results:")
    print(f"   Status: {result['status']} - {result['message']}")
    print(f"   Objective value: {result['objective']:.4f}")
    print(f"   Assigned tasks: {len(result['assigned'])}/{n_tasks}")
    print(f"   Dropped tasks: {len(result['dropped'])}/{n_tasks}")

    # Show assignments
    if len(result['assigned']) > 0:
        print("\n✅ Task Assignments:")
        for drone_id, task_id in result['assigned']:
            dist = d_bar[drone_id, task_id]
            energy = e[drone_id, task_id]
            priority = a_bar[task_id]
            print(f"   Drone {drone_id} → Task {task_id}: "
                  f"dist={dist:.3f}, energy={energy:.0f}Wh, priority={priority:.3f}")

    # Show dropped tasks
    if len(result['dropped']) > 0:
        print("\n❌ Dropped Tasks:")
        for task_id in result['dropped']:
            print(f"   Task {task_id}: priority={a_bar[task_id]:.3f}")

    # Validate solution
    print("\n" + "=" * 70)
    print("Validating solution...")
    print("=" * 70)

    data = {
        'capacity': Q,
        'e': e,
        'w': w
    }

    try:
        validate_solution(result, data, battery_limit)
        print("✅ All constraints satisfied!")
    except AssertionError as ex:
        print(f"❌ Validation failed: {ex}")

    # Show workload distribution
    print("\n📊 Workload Distribution:")
    x_bin = result['x_bin']
    for i in range(n_drones):
        assigned_count = x_bin[i].sum()
        energy_used = (e[i] * x_bin[i]).sum()
        utilization = energy_used / ((1-SIGMA) * E[i]) * 100
        print(f"   Drone {i}: {assigned_count}/{Q[i]} tasks, "
              f"{energy_used:.0f}/{int((1-SIGMA)*E[i])}Wh ({utilization:.1f}%)")

    # Show assignment matrix
    print("\n📋 Assignment Matrix (x_ij):")
    print("   ", end="")
    for j in range(n_tasks):
        print(f"T{j} ", end="")
    print()
    for i in range(n_drones):
        print(f"D{i}: ", end="")
        for j in range(n_tasks):
            if x_bin[i, j] == 1:
                print(" ✓ ", end="")
            elif w[i, j] == 0:
                print(" ✗ ", end="")
            else:
                print(" · ", end="")
        print()

    print("\n" + "=" * 70)
    print("Test completed successfully!")
    print("=" * 70)
