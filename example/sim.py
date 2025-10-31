# Import required modules for drone simulation
from dronesim import Simulator
from dronesim.energy import WattHour
from dronesim.energy.battery import BatteryStatus
from dronesim.geo import GeoPoint, Latitude, Longitude
from dronesim.mission import DeliveryTask
from dronesim.unit import ClockTime, Kilometer
from dronesim.vehicles import DeliveryDrone


class DroneAssignmentsProblemSimulator(Simulator[DeliveryDrone, DeliveryTask]):
    target_city : str
    target_center: GeoPoint
    target_geo_range: Kilometer

    def __init__(self):
        super().__init__()
        self.target_city = "Metropolitian"

        target_res_lat = Latitude(19.176269)  # Example latitude for restaurant
        target_res_lon = Longitude(72.836721)  # Example longitude for restaurant
        self.target_center = GeoPoint(target_res_lat, target_res_lon)

        self.target_geo_range = Kilometer(50)  # 50 km range

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
        pickup_time = ClockTime.from_str(row[columns.index("Time_Order_picked")])
        id = int(row[columns.index("ID")], 16)
        return DeliveryTask(
            origin=origin,
            destination=destination,
            order_time=request_time,
            pickup_time=pickup_time,
            id=id,
        )


    def make_vehicle(self):
        """Generate delivery drone instances for the simulation fleet."""
        for _ in range(200):
            yield DeliveryDrone(
                pos = self.target_center,
                battery= BatteryStatus(WattHour(2000), WattHour(2000)),
                max_task_queue_size=3
            )
        return None

    def results(self):
        """Generate final simulation results and performance metrics."""
        pass

    def sim_update(self, dt, now):
        """Update the simulation state by a time step."""


    def sim_post_update(self, dt, now):
        """Perform any necessary operations after the simulation update."""
        pass

class NearestDroneAssignmentStrategy(DroneAssignmentsProblemSimulator):
    current: int
    def __init__(self):
        super().__init__()
        self.current = 0

    def results(self):
        """Generate final simulation results and performance metrics."""
        pass

    def sim_update(self, dt, now):
        """Update the simulation state by a time step."""
        for task in self.get_pending_tasks():
            self.current = (self.current + 1) % len(self.get_vehicles())
            if not self.get_vehicles()[self.current].assign(task):
                break

    def sim_post_update(self, dt, now):
        """Perform any necessary operations after the simulation update."""
        pass

sim = NearestDroneAssignmentStrategy()
sim.run("example/test.csv", lambda task: task.order_time, j = 1, batch_size=100)
