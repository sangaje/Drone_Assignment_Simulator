# Import required modules for drone simulation
from dronesim import Simulator
from dronesim.energy import WattHour
from dronesim.energy.battery import BatteryStatus
from dronesim.geo import GeoPoint, Latitude, Longitude
from dronesim.mission import DeliveryTask
from dronesim.unit import ClockTime, Hour, Kilometer, KilometersPerHour, Minute, Time, Watt
from dronesim.vehicles import DeliveryDrone

DATA_CSV_FILE = "./example/train.csv"
EXPECT_CSV_FILE = "./example/train.csv"
CLUSTER_DATA_FILE = "./example/train.csv"

# DATA_CSV_FILE = "./test.csv"
# EXPECT_CSV_FILE = "./Sample_Submission.csv"
# CLUSTER_DATA_FILE = "./train.csv"

N_CLUSTERS = 10
WAITING_TIME = Minute(0.1)
DT = Minute(0.1)
DRONE_COUNT = 180

J = 1
BATCH_SIZE = 100

DRONE_VELOCITY = KilometersPerHour(60)
BATTERY_CAPACITY = WattHour(5000)
BATTERY_CURRENT = WattHour(5000)

IDLE_POWER = Watt(10)
VTOL_POWER = Watt(500)
TRAINSIT_POWER = Watt(500)

TASK_QUEUE_PER_DRONE=4
DELVIERYS_PER_CHARGE=0

print("All modules imported successfully!")

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
# RHO = (ALPHA + BETA + GAMMA)  # Balanced approach
RHO = 1

TASK_BATCH_SIZE = 10

# Option 2: Task-specific penalty based on characteristics
# rho_j can be calculated per task based on:
# - Task priority/urgency
# - Customer tier/service level
# - Delivery time window constraints
# - Distance from depot
#
# Example implementations:
# rho_j = base_penalty * priority_multiplier
# rho_j = base_penalty * (1 + urgency_factor)
# rho_j = base_penalty * (2.0 if is_premium_customer else 1.0)

# Recommended Drop Penalty Strategies:
# 1. Mission-Critical: rho = 100 * max(alpha, beta, gamma) - virtually never drop
# 2. Balanced: rho = 2 * (alpha + beta + gamma) - drop only clearly infeasible tasks
# 3. Efficiency-First: rho = 0.5 * (alpha + beta + gamma) - aggressively drop inefficient tasks

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp


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

# Define the main simulation class for drone assignments
class DroneAssignmentsProblemSimulator(Simulator[DeliveryDrone, DeliveryTask]):
    target_city : str
    target_center: GeoPoint
    target_geo_range: Kilometer
    drone_count: int
    cluster_data_path: str
    cluster: int

    def __init__(self,cluster_data_path: str, drone_count: int = 50, cluster:int =1) -> None:
        super().__init__()
        self.target_city = "Metropolitian"

        self.cluster_data_path = cluster_data_path
        self.cluster = cluster
        self._is_clustering_done = False

        target_res_lat = Latitude(12.959861)  # Example latitude for restaurant
        target_res_lon = Longitude(77.579225)  # Example longitude for restaurant
        self.target_center = GeoPoint(target_res_lat, target_res_lon)

        self.target_geo_range = Kilometer(50)  # 50 km range
        self.drone_count = drone_count

    def make_task(self, columns: list[str], row: list[str]) -> DeliveryTask | None:
        """Create a delivery task from CSV data row."""
        if "NaN" in row:
            return None

        restaurant_lat = Latitude(float(row[columns.index("Restaurant_latitude")]))
        restaurant_lon = Longitude(float(row[columns.index("Restaurant_longitude")]))

        delivery_lat = Latitude(float(row[columns.index("Delivery_location_latitude")]))
        delivery_lon = Longitude(float(row[columns.index("Delivery_location_longitude")]))


        origin = GeoPoint(
            restaurant_lat, restaurant_lon
        )
        destination = GeoPoint(
            delivery_lat, delivery_lon
        )
        if origin.distance_to(self.target_center) > self.target_geo_range:
            return None
        if destination.distance_to(self.target_center) > self.target_geo_range:
            return None

        request_time = ClockTime.from_str(row[columns.index("Time_Orderd")])
        if request_time < Hour(3):
            request_time += Hour(24)

        pickup_time = ClockTime.from_str(row[columns.index("Time_Order_picked")])
        id = int(row[columns.index("ID")], 16)
        return DeliveryTask(
            origin=origin,
            destination=destination,
            order_time=request_time,
            pickup_time=pickup_time,
            id=id,
        )

    def _get_bases(self, cluster:int) -> list[tuple[GeoPoint, float]]:
        """Get clustering bases using simple K-means instead of KMedoids to avoid haversine issues."""
        if self._is_clustering_done:
            return self._cached_bases

        try:
            import numpy as np
            from sklearn.cluster import KMeans

            tasks = self.parse_task_data(self.cluster_data_path, key=None)

            # Extract lat/lon coordinates
            points = np.array([[float(task.origin.latitude), float(task.origin.longitude)] for task in tasks])
            print(f"Clustering {len(points)} points into {self.cluster} clusters")

            # Use simple K-means with Euclidean distance (good enough for small geographic areas)
            kmeans = KMeans(n_clusters=min(self.cluster, len(points)), random_state=42, n_init=10)
            labels = kmeans.fit_predict(points)
            centers = kmeans.cluster_centers_

            retval = []
            for i, center in enumerate(centers):
                count = sum(1 for label in labels if label == i)
                if count > 0:  # Only include clusters with actual points
                    latitude = Latitude.from_si(center[0])
                    longitude = Longitude.from_si(center[1])
                    retval.append((GeoPoint(latitude, longitude), count))

            self._cached_bases = retval
            self._is_clustering_done = True
            print(f"Successfully created {len(retval)} cluster bases: ")
            for base, count in retval:
                print(f'({base.latitude}, {base.longitude}) - Store Count: {count}')
            return retval

        except Exception as e:
            print(f"Clustering failed: {e}")
            # Simple fallback: create evenly spaced points around the center
            retval = []
            for i in range(self.cluster):
                # Create points in a circle around the center
                angle = (2 * np.pi * i) / self.cluster
                offset_lat = 0.01 * np.cos(angle)  # Small offset ~1km
                offset_lon = 0.01 * np.sin(angle)

                lat = float(self.target_center.latitude) + offset_lat
                lon = float(self.target_center.longitude) + offset_lon

                retval.append((GeoPoint(Latitude(lat), Longitude(lon)), 100))  # Assume equal distribution

            self._cached_bases = retval
            self._is_clustering_done = True
            return retval

    def make_vehicle(self):
        """Generate delivery drone instances for the simulation fleet."""
        try:
            bases = self._get_bases(self.cluster)
            total_tasks = sum(count for _, count in bases)

            if total_tasks == 0:
                # Fallback: create all drones at center if clustering fails
                for i in range(self.drone_count):
                    yield DeliveryDrone(
                        pos=self.target_center,
                        velocity=DRONE_VELOCITY,
                        battery=BatteryStatus(
                            capacity=BATTERY_CAPACITY,
                            current=BATTERY_CURRENT
                        ),
                        power_idle=IDLE_POWER,
                        power_vtol=VTOL_POWER,
                        power_transit=TRAINSIT_POWER,
                        deliveries_per_charge=DELVIERYS_PER_CHARGE,
                        max_task_queue_size=TASK_QUEUE_PER_DRONE
                    )
                return

            drones_created = 0
            last_base = self.target_center

            for base, count in bases:
                # Calculate proportional drone count for this base
                base_drone_count = int((count / total_tasks) * self.drone_count)

                for i in range(base_drone_count):
                    if drones_created < self.drone_count:
                        yield DeliveryDrone(
                            pos=base,
                            velocity=DRONE_VELOCITY,
                            battery=BatteryStatus(
                                capacity=BATTERY_CAPACITY,
                                current=BATTERY_CURRENT
                            ),
                            power_idle=IDLE_POWER,
                            power_vtol=VTOL_POWER,
                            power_transit=TRAINSIT_POWER,
                            deliveries_per_charge=DELVIERYS_PER_CHARGE,
                            max_task_queue_size=TASK_QUEUE_PER_DRONE
                        )
                        drones_created += 1

                last_base = base

            # Create remaining drones at the last base
            while drones_created < self.drone_count:
                yield DeliveryDrone(
                    pos=last_base,
                    velocity=DRONE_VELOCITY,
                    battery=BatteryStatus(
                        capacity=BATTERY_CAPACITY,
                        current=BATTERY_CURRENT
                    ),
                    power_idle=IDLE_POWER,
                    power_vtol=VTOL_POWER,
                    power_transit=TRAINSIT_POWER,
                    deliveries_per_charge=DELVIERYS_PER_CHARGE,
                    max_task_queue_size=TASK_QUEUE_PER_DRONE
                )
                drones_created += 1

        except Exception as e:
            print(f"Warning: Clustering failed ({e}), using fallback drone creation")
            # Fallback: create all drones at center
            for i in range(self.drone_count):
                yield DeliveryDrone(
                    pos=self.target_center,
                    velocity=DRONE_VELOCITY,
                    battery=BatteryStatus(
                        capacity=BATTERY_CAPACITY,
                        current=BATTERY_CURRENT
                    ),
                    power_idle=IDLE_POWER,
                    power_vtol=VTOL_POWER,
                    power_transit=TRAINSIT_POWER,
                    deliveries_per_charge=DELVIERYS_PER_CHARGE,
                    max_task_queue_size=TASK_QUEUE_PER_DRONE
                )

        return None

class MILPDroneAssignmentStrategy(DroneAssignmentsProblemSimulator):
    """MILP-Based Optimal Drone Assignment Strategy using Mixed-Integer Linear Programming.
    
    This strategy formulates the drone-task assignment problem as a MILP optimization
    problem and finds the globally optimal solution considering multiple objectives:
    - Minimize total flight distance
    - Minimize energy consumption
    - Maximize task priority satisfaction
    - Minimize task drops
    
    The optimizer respects all physical constraints:
    - Battery capacity limits with safety margins
    - Drone task capacity limits
    - Operational range constraints
    """

    def __init__(self, drone_count: int = 50, waiting_time: Time = Minute(1)):
        super().__init__(CLUSTER_DATA_FILE, drone_count, N_CLUSTERS)
        self.current = waiting_time
        self.waiting_time = waiting_time

    def sim_update(self, dt, now):
        """Update simulation by solving MILP optimization for pending tasks."""
        if now < self.current:
            if self.pending_tasks_count < TASK_BATCH_SIZE:
                return
        else:
            self.current += self.waiting_time


        available_drones = [drone for drone in self.get_vehicles() if drone.is_operational()]
        if len(available_drones) == 0:
            return
        tasks = []

        for task in self.get_pending_tasks():
            tasks.append(task)

        if len(tasks) == 0:
            return

        # Prepare optimization data
        n_drones = len(available_drones)
        n_tasks = len(tasks)
        # print(n_drones, n_tasks)

        # Build parameter matrices
        d_bar, e_bar, a_bar, e, w, Q, E = self._build_optimization_data(tasks, available_drones)

        # Build and solve MILP problem
        try:
            c, bounds, constraints, ub_x, battery_limit = build_milp_problem(
                n_drones, n_tasks, d_bar, e_bar, a_bar, e, w, Q, E
            )

            result = solve_and_parse(c, bounds, constraints, n_drones, n_tasks)

            # Execute assignments
            self._execute_assignments(result, tasks, available_drones)

            # Handle dropped tasks
            for task_idx in result['dropped']:
                self.failed_to_assign_task(tasks[task_idx])

        except Exception as ex:
            print(f"MILP optimization failed: {ex}")
            # Fallback: mark all tasks as failed
            for task in tasks:
                self.failed_to_assign_task(task)

    def _build_optimization_data(self, tasks: list[DeliveryTask], drones: list[DeliveryDrone]):
        """Build parameter matrices for MILP optimization."""
        n_drones = len(drones)
        n_tasks = len(tasks)

        # Initialize matrices
        d = np.zeros((n_drones, n_tasks))      # Total flight distance
        e = np.zeros((n_drones, n_tasks))      # Energy consumption (Wh)
        w = np.zeros((n_drones, n_tasks))      # Range feasibility
        priority = np.zeros(n_tasks)           # Task priorities

        Q = np.zeros(n_drones, dtype=int)      # Drone capacity
        E = np.zeros(n_drones)                 # Battery capacity

        # Calculate drone parameters
        for i, drone in enumerate(drones):
            if not drone.is_operational():
                Q[i] = 0
                E[i] = 0.0
            else:
                Q[i] = TASK_QUEUE_PER_DRONE
                E[i] = float(drone.battery.current)

        # Calculate task parameters
        for j, task in enumerate(tasks):
            # Simple priority: prefer earlier order times
            priority[j] = task.priority

            for i, drone in enumerate(drones):
                d[i, j] = float(drone.route_remainder(task)[0]) + 1e6
                e[i, j] = d[i, j] * float(drone.power_transit) / float(drone.velocity)

                # Range constraint: check if drone can reach task
                max_range = float(drone.velocity) * float(drone.battery.current) / float(drone.power_transit) * 0.8
                w[i, j] = 1 if d[i, j] <= max_range else 0

        # Normalize parameters (simple and safe)
        d_max = np.max(d) if d.size > 0 else 1.0
        e_max = np.max(E) if E.size > 0 else 1.0
        a_max = np.max(priority) if priority.size > 0 else 1.0

        d_bar = d / d_max
        e_bar = e / e_max
        a_bar = priority / a_max

        return d_bar, e_bar, a_bar, e, w, Q, E

    def _execute_assignments(self, result, tasks, drones):
        """Execute the optimized assignments to drones."""
        for drone_idx, task_idx in result['assigned']:
            drone = drones[drone_idx]
            task = tasks[task_idx]

            if not drone.assign(task):
                self.failed_to_assign_task(task)

    def sim_post_update(self, dt, now):
        """Perform any necessary operations after the simulation update."""
        pass

WAITING_TIME = Minute(1)
TASK_QUEUE_PER_DRONE=4
DELVIERYS_PER_CHARGE=4

# Run MILP-Based Optimal Drone Assignment Strategy simulation
milp_sim = MILPDroneAssignmentStrategy(DRONE_COUNT, waiting_time=WAITING_TIME)
milp_sim.run(DATA_CSV_FILE, lambda task: task.order_time, j = J, dt = DT, batch_size=BATCH_SIZE)
p_milp = SimPlot(milp_sim, EXPECT_CSV_FILE)
p_milp.task_processing_times()
p_milp.expected_time_analysis()
p_milp.deviation_in_time_taken()
p_milp.task_speed()
p_milp.deviation_speed()
p_milp.battery_usage()
p_milp.show()
