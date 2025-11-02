# Import required modules for drone simulation
from dronesim import Simulator
from dronesim.energy import WattHour
from dronesim.energy.battery import BatteryStatus
from dronesim.geo import GeoPoint, Latitude, Longitude
from dronesim.mission import DeliveryTask
from dronesim.unit import ClockTime, Kilometer, KilometersPerHour, Minute, Time
from dronesim.vehicles import DeliveryDrone

DATA_CSV_FILE = "./train.csv"
EXPECT_CSV_FILE = "./train.csv"
CLUSTER_DATA_FILE = "./train.csv"
N_CLUSTERS = 3
WATING_TIME = Minute(1)
DRONE_COUNT = 200
J = 1
BATCH_SIZE = 200
TASK_QUEUE_PER_DRONE = 1


class DroneAssignmentsProblemSimulator(Simulator[DeliveryDrone, DeliveryTask]):
    target_City: str
    target_center: GeoPoint
    target_geo_range: Kilometer
    drone_count: int
    cluster_data_path: str
    cluster: int

    def __init__(
        self, cluster_data_path: str, drone_count: int = 50, cluster: int = 1
    ) -> None:
        super().__init__()
        self.target_City = "Metropolitian"

        self.cluster_data_path = cluster_data_path
        self.cluster = cluster
        self._is_clustering_done = False

        target_res_lat = Latitude(19.176269)  # Example latitude for restaurant
        target_res_lon = Longitude(72.836721)  # Example longitude for restaurant
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
        delivery_lon = Longitude(
            float(row[columns.index("Delivery_location_longitude")])
        )

        origin = GeoPoint(restaurant_lat, restaurant_lon)
        destination = GeoPoint(delivery_lat, delivery_lon)
        if origin.distance_to(self.target_center) > self.target_geo_range:
            return None
        if destination.distance_to(self.target_center) > self.target_geo_range:
            return None

        request_time = ClockTime.from_str(row[columns.index("Time_Orderd")])
        pickup_time = ClockTime.from_str(row[columns.index("Time_Order_picked")])
        id = int(row[columns.index("ID")], 16)
        return DeliveryTask(
            origin=origin,
            destination=destination,
            order_time=request_time,
            pickup_time=pickup_time,
            id=id,
        )

    def _get_bases(self, cluster: int) -> list[tuple[GeoPoint, float]]:
        """Get clustering bases using simple K-means instead of KMedoids to avoid haversine issues."""
        if self._is_clustering_done:
            return self._cached_bases

        try:
            import numpy as np
            from sklearn.cluster import KMeans

            tasks = self.parse_task_data(self.cluster_data_path, key=None)


            # Extract lat/lon coordinates
            points = np.array(
                [
                    [float(task.origin.latitude), float(task.origin.longitude)]
                    for task in tasks
                ]
            )
            print(f"Clustering {len(points)} points into {self.cluster} clusters")

            # Use simple K-means with Euclidean distance (good enough for small geographic areas)
            kmeans = KMeans(
                n_clusters=min(self.cluster, len(points)), random_state=42, n_init=10
            )
            labels = kmeans.fit_predict(points)
            centers = kmeans.cluster_centers_

            retval = []
            for i, center in enumerate(centers):
                count = sum(1 for label in labels if label == i)
                if count > 0:  # Only include clusters with actual points
                    latitude = Latitude(center[0])
                    longitude = Longitude(center[1])
                    retval.append((GeoPoint(latitude, longitude), count))

            self._cached_bases = retval
            self._is_clustering_done = True

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

                retval.append(
                    (GeoPoint(Latitude(lat), Longitude(lon)), 100)
                )  # Assume equal distribution

            self._cached_bases = retval
            self._is_clustering_done = True
            return retval
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
                        velocity=KilometersPerHour(80),
                        battery=BatteryStatus(WattHour(2000), WattHour(2000)),
                        max_task_queue_size=TASK_QUEUE_PER_DRONE,
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
                            velocity=KilometersPerHour(80),
                            battery=BatteryStatus(WattHour(2000), WattHour(2000)),
                            max_task_queue_size=TASK_QUEUE_PER_DRONE,
                        )
                        drones_created += 1

                last_base = base

            # Create remaining drones at the last base
            while drones_created < self.drone_count:
                yield DeliveryDrone(
                    pos=last_base,
                    velocity=KilometersPerHour(80),
                    battery=BatteryStatus(WattHour(2000), WattHour(2000)),
                    max_task_queue_size=TASK_QUEUE_PER_DRONE,
                )
                drones_created += 1

        except Exception as e:
            print(f"Warning: Clustering failed ({e}), using fallback drone creation")
            # Fallback: create all drones at center
            for i in range(self.drone_count):
                yield DeliveryDrone(
                    pos=self.target_center,
                    velocity=KilometersPerHour(80),
                    battery=BatteryStatus(WattHour(2000), WattHour(2000)),
                    max_task_queue_size=TASK_QUEUE_PER_DRONE,
                )

        return None


from dronesim.simulator import analyze_task_processing_times


class SimPlot:
    def __init__(self, simulator: Simulator, sample_submission_csv: str | None = None):
        self.simulator = simulator
        self.sample_submission_csv = sample_submission_csv

    def deviation_in_time_taken(self):
        import csv

        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플

        task_data = []
        matched_count = 0
        total_tasks = len(tasks)

        print(f"Total simulation tasks: {total_tasks}")

        with open(self.sample_submission_csv) as f:
            reader = csv.reader(f)
            columns = next(reader)
            for row in reader:
                row = [cell.strip() for cell in row]
                row_id = int(row[columns.index("ID")], 16)

                # Find matching task
                task = None
                for t in tasks:
                    if t.id == row_id:
                        task = t
                        matched_count += 1
                        break

                if task is None:
                    continue

                # Check if task has required timing data
                if task.start_at is None or task.completed_at is None:
                    print(
                        f"Task {task.id} missing timing data: start_at={task.start_at}, completed_at={task.completed_at}"
                    )
                    continue

                # Calculate simulation processing time (in minutes)
                sim_processing_time = float(task.completed_at) - float(task.start_at)
                # Get expected time from CSV (already in minutes)
                expected_time = float(row[columns.index("Time_taken(min)")]) * 60

                # Calculate deviation (expected - actual)
                time_deviation = expected_time - sim_processing_time

                task_data.append(
                    {
                        "start_time": float(task.start_at),
                        "processing_time": time_deviation,
                    }
                )

        print(f"Matched tasks: {matched_count}")
        print(f"Valid data points: {len(task_data)}")

        if len(task_data) == 0:
            print("No valid data found - checking first few tasks...")
            for i, task in enumerate(tasks[:5]):
                print(
                    f"Task {i}: id={task.id}, start_at={task.start_at}, completed_at={task.completed_at}"
                )

        return analyze_task_processing_times(task_data, "Time Deviation Analysis")

    def expected_time_analysis(self):
        """Analyze expected processing times from the sample submission CSV."""
        import csv

        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플

        task_data = []
        matched_count = 0
        total_tasks = len(tasks)

        with open(self.sample_submission_csv) as f:
            reader = csv.reader(f)
            columns = next(reader)
            for row in reader:
                row = [cell.strip() for cell in row]
                row_id = int(row[columns.index("ID")], 16)

                # Find matching task
                task = None
                for t in tasks:
                    if t.id == row_id:
                        task = t
                        matched_count += 1
                        break

                if task is None:
                    continue

                # Check if task has required timing data
                if task.start_at is None or task.completed_at is None:
                    continue

                # Get expected time from CSV (in minutes) and convert to seconds
                expected_time_minutes = float(row[columns.index("Time_taken(min)")])
                expected_time_seconds = (
                    expected_time_minutes * 60
                )  # Convert minutes to seconds

                task_data.append(
                    {
                        "start_time": float(task.start_at),
                        "processing_time": expected_time_seconds,  # Now in seconds for consistency
                    }
                )

        print(f"Expected time data points: {len(task_data)}")
        return analyze_task_processing_times(task_data, "Expected Time Analysis")

    def task_processing_times(self):
        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플 언패킹 제거
        task_data = []
        for task in tasks:
            if task.start_at is not None and task.completed_at is not None:
                start_time = float(task.start_at)
                processing_time = float(task.completed_at) - start_time

                task_data.append(
                    {"start_time": start_time, "processing_time": processing_time}
                )

        return analyze_task_processing_times(task_data, "Task Processing Time Analysis")

class FairDroneAssignmentStrategy(DroneAssignmentsProblemSimulator):
    drone_i: int
    current: Time

    def __init__(self, drone_count: int = 50, wating_time: Time = Minute(1)):
        super().__init__(CLUSTER_DATA_FILE, drone_count, N_CLUSTERS)
        self.drone_i = 0
        self.current = wating_time
        self.wating_time = wating_time

    def sim_update(self, dt, now):
        """Update the simulation state by a time step."""
        if now < self.current:
            return
        self.current += self.wating_time

        for task in self.get_pending_tasks():
            if not self.get_vehicles()[self.drone_i].assign(task):
                break
            self.drone_i = (self.drone_i + 1) % len(self.get_vehicles())

    def sim_post_update(self, dt, now):
        """Perform any necessary operations after the simulation update."""
        pass

# Run Fair Drone Assignment Strategy simulation
fair_sim = FairDroneAssignmentStrategy(DRONE_COUNT, wating_time=WATING_TIME)
fair_sim.run(DATA_CSV_FILE, lambda task: task.order_time, j = J, batch_size=BATCH_SIZE)
p = SimPlot(fair_sim, EXPECT_CSV_FILE)
