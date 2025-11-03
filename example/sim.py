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

N_CLUSTERS = 10
WATING_TIME = Minute(0.1)
DT = Minute(0.1)
DRONE_COUNT = 200

J = 1
BATCH_SIZE = 100

DRONE_VELOCITY = KilometersPerHour(60)
BATTERY = BatteryStatus(
    capacity=WattHour(50000),
    current=WattHour(50000)
)
IDLE_POWER = Watt(0)
VTOL_POWER = Watt(0)
TRAINSIT_POWER = Watt(500)

TASK_QUEUE_PER_DRONE=2
DELVIERYS_PER_CHARGE=2





print("All modules imported successfully!")

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
                        battery=BATTERY,
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
                            battery=BATTERY,
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
                    battery=BATTERY,
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
                    battery=BATTERY,
                    power_idle=IDLE_POWER,
                    power_vtol=VTOL_POWER,
                    power_transit=TRAINSIT_POWER,
                    deliveries_per_charge=DELVIERYS_PER_CHARGE,
                    max_task_queue_size=TASK_QUEUE_PER_DRONE
                )

        return None

class OptimalDistanceAssignmentStrategy(DroneAssignmentsProblemSimulator):
    """Nearest Drone Assignment Strategy implementing Greedy Distance-Based Algorithm.
    Assigns each task to the closest available drone based on geographical proximity.
    """

    def __init__(self, drone_count: int = 50, wating_time: Time = Minute(1)):
        super().__init__(CLUSTER_DATA_FILE, drone_count, N_CLUSTERS)
        self.current = wating_time
        self.wating_time = wating_time

    def sim_update(self, dt, now):
        """Update the simulation state by assigning tasks to nearest available drones."""
        if now < self.current:
            return
        self.current += self.wating_time

        for task in self.get_pending_tasks():
            # Find the nearest available drone to the task's origin
            if not self._find_optimal_drone(task):
                self.failed_to_assign_task(task)
                break

    def _find_optimal_drone(self, task: DeliveryTask):
        """Find the drone with minimum distance to the task's origin."""
        # Only consider operational drones (not full and has battery)
        available_drones = [drone for drone in self.get_vehicles() if drone.is_operational()]
        if len(available_drones) == 0:
            return False

        drone_distances = []

        for drone in available_drones:
            distance, last_point = drone.route_remainder(task)
            d_distance = distance - drone.route_remainder()[0]
            drone_distances.append((drone, distance, d_distance))

        # Sort by distance (ascending) - using float conversion for sorting
        drone_distances.sort(key=lambda x: float(x[1]))

        # Try to assign to drones in order of proximity
        for drone, _, d_distance in drone_distances:
            if drone.assign(task):
                return True
        return False

    def sim_post_update(self, dt, now):
        """Perform any necessary operations after the simulation update."""
        pass

optimal_sim = OptimalDistanceAssignmentStrategy(DRONE_COUNT, wating_time=WATING_TIME)
optimal_sim.run(DATA_CSV_FILE, lambda task: task.order_time, j = J, dt = DT, batch_size=BATCH_SIZE)
