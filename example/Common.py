"""Common classes and utilities for drone assignment simulation.

This module provides base classes and plotting utilities for drone delivery
simulations, including the DroneAssignmentsProblemSimulator and SimPlot classes.
"""

from Config import (
    BATTERY_CAPACITY,
    BATTERY_CURRENT,
    DELVIERYS_PER_CHARGE,
    DRONE_VELOCITY,
    IDLE_POWER,
    OPERATIONAL_BATTERY_PERCENTAEG,
    PER_PACAGE_POWER,
    TASK_QUEUE_PER_DRONE,
    TRAINSIT_POWER,
    VTOL_POWER,
)
from dronesim import Simulator
from dronesim.energy.battery import BatteryStatus
from dronesim.energy.unit import Energy
from dronesim.geo import GeoPoint, Latitude, Longitude
from dronesim.mission import DeliveryTask
from dronesim.mission.task_delivery import DeliveryState
from dronesim.simulator import (
    analyze_task_processing_speed,
    analyze_task_processing_times,
    analyze_vehicle_battery_consumption,
)
from dronesim.unit import ClockTime, Kilometer, Minute, Time
from dronesim.unit.unit_time import Hour
from dronesim.vehicles import DeliveryDrone
import matplotlib.pyplot as plt


class DroneAssignmentsProblemSimulator(Simulator[DeliveryDrone, DeliveryTask]):
    """Simulator for drone delivery assignment problems.

    This class handles the simulation of drone delivery operations, including
    clustering of delivery locations and management of drone fleets.

    Attributes:
        target_city: Name of the target city for simulation.
        target_center: Geographic center point of the target area.
        target_geo_range: Maximum range from center for task acceptance.
        drone_count: Number of drones in the fleet.
        cluster_data_path: Path to clustering data file.
        cluster: Number of clusters for base distribution.
    """

    target_city: str
    target_center: GeoPoint
    target_geo_range: Kilometer
    drone_count: int
    cluster_data_path: str
    cluster: int

    def __init__(
        self, cluster_data_path: str, drone_count: int = 50, cluster: int = 1
    ) -> None:
        """Initialize the drone assignment simulator.

        Args:
            cluster_data_path: Path to the CSV file containing clustering data.
            drone_count: Number of drones to create in the simulation.
            cluster: Number of base clusters to create.
        """
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

    def _get_bases(self, cluster: int) -> list[tuple[GeoPoint, float]]:
        """Get clustering bases using K-means clustering.

        Uses simple K-means instead of KMedoids to avoid haversine issues.

        Args:
            cluster: Number of clusters to create.

        Returns:
            List of tuples containing (GeoPoint, count) for each base.
        """
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

            retval: list[tuple[GeoPoint, int]] = []
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
                print(f"({base.latitude}, {base.longitude}) - Store Count: {count}")
            return retval  # noqa: TRY300

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

    def make_vehicle(self):
        """Generate delivery drone instances for the simulation fleet."""
        try:
            bases = self._get_bases(self.cluster)
            total_tasks = sum(count for _, count in bases)

            if total_tasks == 0:
                # Fallback: create all drones at center if clustering fails
                for _ in range(self.drone_count):
                    yield DeliveryDrone(
                        pos=self.target_center,
                        velocity=DRONE_VELOCITY,
                        battery=BatteryStatus(
                            capacity=BATTERY_CAPACITY, current=BATTERY_CURRENT
                        ),
                        power_idle=IDLE_POWER,
                        power_vtol=VTOL_POWER,
                        power_transit=TRAINSIT_POWER,
                        power_per_pakage=PER_PACAGE_POWER,
                        deliveries_per_charge=DELVIERYS_PER_CHARGE,
                        max_task_queue_size=TASK_QUEUE_PER_DRONE,
                        operational_battery_percentage=OPERATIONAL_BATTERY_PERCENTAEG,
                    )
                return

            drones_created = 0
            last_base = self.target_center

            for base, count in bases:
                # Calculate proportional drone count for this base
                base_drone_count = int((count / total_tasks) * self.drone_count)

                for _ in range(base_drone_count):
                    if drones_created < self.drone_count:
                        yield DeliveryDrone(
                            pos=base,
                            velocity=DRONE_VELOCITY,
                            battery=BatteryStatus(
                                capacity=BATTERY_CAPACITY, current=BATTERY_CURRENT
                            ),
                            power_idle=IDLE_POWER,
                            power_vtol=VTOL_POWER,
                            power_transit=TRAINSIT_POWER,
                            power_per_pakage=PER_PACAGE_POWER,
                            deliveries_per_charge=DELVIERYS_PER_CHARGE,
                            max_task_queue_size=TASK_QUEUE_PER_DRONE,
                            operational_battery_percentage=OPERATIONAL_BATTERY_PERCENTAEG,
                        )
                        drones_created += 1

                last_base = base

            # Create remaining drones at the last base
            while drones_created < self.drone_count:
                yield DeliveryDrone(
                    pos=last_base,
                    velocity=DRONE_VELOCITY,
                    battery=BatteryStatus(
                        capacity=BATTERY_CAPACITY, current=BATTERY_CURRENT
                    ),
                    power_idle=IDLE_POWER,
                    power_vtol=VTOL_POWER,
                    power_transit=TRAINSIT_POWER,
                    power_per_pakage=PER_PACAGE_POWER,
                    deliveries_per_charge=DELVIERYS_PER_CHARGE,
                    max_task_queue_size=TASK_QUEUE_PER_DRONE,
                    operational_battery_percentage=OPERATIONAL_BATTERY_PERCENTAEG,
                )
                drones_created += 1

        except Exception as e:
            print(f"Warning: Clustering failed ({e}), using fallback drone creation")
            # Fallback: create all drones at center
            for _ in range(self.drone_count):
                yield DeliveryDrone(
                    pos=self.target_center,
                    velocity=DRONE_VELOCITY,
                    battery=BatteryStatus(
                        capacity=BATTERY_CAPACITY, current=BATTERY_CURRENT
                    ),
                    power_idle=IDLE_POWER,
                    power_vtol=VTOL_POWER,
                    power_transit=TRAINSIT_POWER,
                    power_per_pakage=PER_PACAGE_POWER,
                    deliveries_per_charge=DELVIERYS_PER_CHARGE,
                    max_task_queue_size=TASK_QUEUE_PER_DRONE,
                    operational_battery_percentage=OPERATIONAL_BATTERY_PERCENTAEG,
                )

        return None




class SimPlot:
    """Plotting utilities for simulation analysis.

    This class provides methods for analyzing and visualizing simulation results,
    including task processing times, speed deviations, and battery usage.

    Attributes:
        simulator: The simulator instance to analyze.
        sample_submission_csv: Optional path to CSV file for comparison.
    """

    def __init__(self, simulator: Simulator, sample_submission_csv: str | None = None):
        """Initialize the simulation plotter.

        Args:
            simulator: Simulator instance containing results to analyze.
            sample_submission_csv: Optional path to CSV for expected values.
        """
        self.simulator = simulator
        self.sample_submission_csv = sample_submission_csv

    def deviation_in_time_taken(self):
        """Analyze deviation between expected and actual task processing times.

        Returns:
            Analysis result from analyze_task_processing_times.
        """
        import csv

        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플

        task_data = []
        traffic_data = []
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
                        f"Task {task.id} missing timing data: "
                        f"start_at={task.start_at}, "
                        f"completed_at={task.completed_at}"
                    )
                    continue

                start_time = float(task.start_at)

                # Calculate simulation processing time (in minutes)
                sim_processing_time = float(task.completed_at) - start_time
                try:
                    request_time = ClockTime.from_str(row[columns.index("Time_Orderd")])
                    pickup_time = ClockTime.from_str(
                        row[columns.index("Time_Order_picked")]
                    )
                    wating_time = float(pickup_time - request_time)
                except Exception:  # noqa: S110
                    wating_time = float(Minute(10))
                if wating_time < 0:
                    wating_time = -(wating_time + float(Hour(24)))

                expected_time = (
                    float(row[columns.index("Time_taken(min)")]) * 60 + wating_time
                )

                # Calculate deviation (expected - actual)
                time_deviation = expected_time - sim_processing_time

                task_data.append(
                    {"start_time": start_time, "processing_time": time_deviation}
                )
                try:
                    traffic = row[columns.index("Road_traffic_density")]
                    traffic_data.append(
                        {
                            "start_time": float(task.start_at),
                            "congestion_level": traffic,
                        }
                    )
                except Exception:  # noqa: S110
                    pass
        # print(f"Matched tasks: {matched_count}")
        # print(f"Valid data points: {len(task_data)}")

        if len(task_data) == 0:
            print("No valid data found - checking first few tasks...")
            for i, task in enumerate(tasks[:5]):
                print(
                    f"Task {i}: id={task.id}, "
                    f"start_at={task.start_at}, "
                    f"completed_at={task.completed_at}"
                )

        return analyze_task_processing_times(
            task_data, "Time Deviation Analysis", traffic_data
        )

    def expected_time_analysis(self):
        """Analyze expected processing times from the sample submission CSV."""
        import csv

        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플

        task_data = []
        traffic_data = []

        matched_count = 0
        _total_tasks = len(tasks)  # noqa: F841

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
                try:
                    request_time = ClockTime.from_str(row[columns.index("Time_Orderd")])
                    pickup_time = ClockTime.from_str(
                        row[columns.index("Time_Order_picked")]
                    )
                    wating_time = float(pickup_time - request_time)
                except Exception:  # noqa: S110
                    wating_time = float(Minute(10))

                if wating_time < 0:
                    wating_time = -(wating_time + float(Hour(24)))

                expected_time = (
                    float(row[columns.index("Time_taken(min)")]) * 60 + wating_time
                )

                task_data.append(
                    {
                        "start_time": float(task.start_at),
                        "processing_time": expected_time,  # Now in seconds for consistency
                    }
                )
                try:
                    traffic = row[columns.index("Road_traffic_density")]
                    traffic_data.append(
                        {
                            "start_time": float(task.start_at),
                            "congestion_level": traffic,
                        }
                    )
                except Exception:  # noqa: S110
                    pass

        # print(f"Expected time data points: {len(task_data)}")
        return analyze_task_processing_times(
            task_data, "Expected Time Analysis", traffic_data
        )

    def task_processing_times(self):
        """Analyze actual task processing times from simulation.

        Returns:
            Analysis result from analyze_task_processing_times.
        """
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

    def task_speed(self):
        """Analyze task delivery speed from simulation.

        Returns:
            Analysis result from analyze_task_processing_speed.
        """
        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플 언패킹 제거
        task_data = []
        for task in tasks:
            if task.start_at is not None and task.completed_at is not None:
                start_time = float(task.start_at)
                t = float(task.event_time[DeliveryState.SERVICE_DROPOFF]) - float(
                    task.event_time[DeliveryState.SERVICE_PICKUP]
                )
                speed = (float(task.origin.distance_to(task.destination))) / (t)  # km/h

                task_data.append({"start_time": start_time, "speed": speed})

        return analyze_task_processing_speed(
            task_data, "Task Processing Speed Analysis"
        )

    def deviation_speed(self):
        """Analyze deviation between expected and actual delivery speeds.

        Returns:
            Analysis result from analyze_task_processing_speed.
        """
        import csv

        tasks: list[DeliveryTask]
        tasks, _ = self.simulator.results  # 메서드 호출, 튜플

        task_data = []
        traffic_data = []
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
                        f"Task {task.id} missing timing data: "
                        f"start_at={task.start_at}, "
                        f"completed_at={task.completed_at}"
                    )
                    continue

                start_time = float(task.start_at)

                # Calculate simulation processing time (in minutes)
                sim_delivery_time = float(
                    task.event_time[DeliveryState.SERVICE_DROPOFF]
                ) - float(task.event_time[DeliveryState.SERVICE_PICKUP])
                # Get expected time from CSV (already in minutes)

                expected_time = float(row[columns.index("Time_taken(min)")]) * 60

                distance = float(task.origin.distance_to(task.destination))
                epceted_speed = distance / expected_time
                sim_speed = distance / sim_delivery_time

                # Calculate deviation (expected - actual)
                speed = sim_speed - epceted_speed

                task_data.append({"start_time": start_time, "speed": speed})
                try:
                    traffic = row[columns.index("Road_traffic_density")]
                    traffic_data.append(
                        {
                            "start_time": float(task.start_at),
                            "congestion_level": traffic,
                        }
                    )
                except Exception:  # noqa: S110
                    pass

        # print(f"Matched tasks: {matched_count}")
        # print(f"Valid data points: {len(task_data)}")

        if len(task_data) == 0:
            print("No valid data found - checking first few tasks...")
            for i, task in enumerate(tasks[:5]):
                print(
                    f"Task {i}: id={task.id}, "
                    f"start_at={task.start_at}, "
                    f"completed_at={task.completed_at}"
                )

        return analyze_task_processing_speed(
            task_data, "Speed Deviation Analysis", traffic_data
        )

    def battery_usage(self):
        """Analyze battery consumption across all vehicles.

        Returns:
            Analysis result from analyze_vehicle_battery_consumption.
        """
        tasks: list[DeliveryTask]
        vehicles: list[DeliveryDrone]
        tasks, vehicles = self.simulator.results  # 메서드 호출, 튜플
        battery_usage_history: list[tuple[Time, Time, Energy]] = []
        vehicle_data = []
        for history in [vehicle.battery_usage_history for vehicle in vehicles]:
            battery_usage_history += history

        for battery_usage in battery_usage_history:
            vehicle_data.append(
                {
                    "start_time": float(battery_usage[0]),
                    "battery_used": float(battery_usage[2]),
                }
            )

        return analyze_vehicle_battery_consumption(
            vehicle_data, "Vehicle Battery Consumption Analysis"
        )

    def show(self):
        """Display all generated plots."""
        plt.show()
