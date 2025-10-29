"""Multi-threaded drone simulation framework.

This module provides enhanced simulation capabilities with multi-threading support
for handling large numbers of drones efficiently. It implements thread-safe
vehicle management, parallel task processing, and real-time simulation updates.
"""

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from queue import Queue
import threading
import time
from typing import Any

from dronesim.unit import Second, Time
from dronesim.vehicles import Vehicle

from .simulator import Simulator


@dataclass
class SimulationConfig:
    """Configuration for threaded simulation."""

    max_workers: int = 4
    update_interval: float = 0.1  # seconds
    batch_size: int = 10
    enable_real_time: bool = True
    log_performance: bool = False


@dataclass
class SimulationMetrics:
    """Performance metrics for simulation monitoring."""

    total_vehicles: int = 0
    active_threads: int = 0
    updates_per_second: float = 0.0
    average_update_time: float = 0.0
    peak_memory_usage: float = 0.0
    simulation_time: float = 0.0
    real_time: float = 0.0
    time_ratio: float = 1.0  # simulation_time / real_time


class ThreadedSimulator(Simulator):
    """Multi-threaded simulation framework for drone operations.

    This simulator extends the base Simulator class with multi-threading
    capabilities to handle large numbers of vehicles efficiently. It provides
    thread-safe operations, parallel processing, and real-time simulation.

    Key Features:
    - Parallel vehicle updates using thread pools
    - Thread-safe vehicle management
    - Real-time simulation with configurable timing
    - Performance monitoring and metrics
    - Batch processing for optimal performance
    - Event-driven architecture for extensibility

    Threading Architecture:
    - Main thread: Simulation control and coordination
    - Worker threads: Vehicle updates and task processing
    - Background thread: Metrics collection and logging
    - Event thread: Real-time event processing (optional)
    """

    def __init__(self, config: SimulationConfig | None = None):
        """Initialize the threaded simulator.

        Args:
            config: Simulation configuration. Uses defaults if None.
        """
        super().__init__()
        self.config = config or SimulationConfig()

        # Thread-safe vehicle management
        self.vehicles: dict[int, Vehicle] = {}
        self.vehicles_lock = threading.RLock()

        # Simulation state
        self.running = False
        self.paused = False
        self.simulation_time = 0.0
        self.start_real_time = 0.0

        # Thread management
        self.executor: ThreadPoolExecutor | None = None
        self.main_thread: threading.Thread | None = None
        self.metrics_thread: threading.Thread | None = None

        # Event system
        self.event_queue: Queue = Queue()
        self.event_handlers: dict[str, list[Callable]] = {}

        # Performance tracking
        self.metrics = SimulationMetrics()
        self.update_times: list[float] = []
        self.last_metrics_update = 0.0

        # Synchronization
        self.pause_event = threading.Event()
        self.shutdown_event = threading.Event()

    def init_data(self):
        """Initialize simulation data and threading infrastructure."""
        # Initialize thread pool
        self.executor = ThreadPoolExecutor(
            max_workers=self.config.max_workers, thread_name_prefix="DroneSimWorker"
        )

        # Start metrics collection if enabled
        if self.config.log_performance:
            self.metrics_thread = threading.Thread(
                target=self._metrics_collector, name="MetricsCollector", daemon=True
            )
            self.metrics_thread.start()

        # Initialize timing
        self.start_real_time = time.time()
        self.simulation_time = 0.0

        # Signal successful initialization
        self.emit_event(
            "simulation_initialized", {"config": self.config, "timestamp": time.time()}
        )

    def init_vehicles(self):
        """Initialize vehicles - to be implemented by subclasses."""
        pass

    def add_vehicle(self, vehicle: Vehicle) -> bool:
        """Add a vehicle to the simulation in a thread-safe manner.

        Args:
            vehicle: Vehicle instance to add.

        Returns:
            True if vehicle was added successfully, False if already exists.
        """
        with self.vehicles_lock:
            if vehicle.id in self.vehicles:
                return False

            self.vehicles[vehicle.id] = vehicle
            self.metrics.total_vehicles = len(self.vehicles)

            self.emit_event(
                "vehicle_added",
                {
                    "vehicle_id": vehicle.id,
                    "total_vehicles": self.metrics.total_vehicles,
                },
            )

            return True

    def remove_vehicle(self, vehicle_id: int) -> bool:
        """Remove a vehicle from the simulation.

        Args:
            vehicle_id: ID of vehicle to remove.

        Returns:
            True if vehicle was removed, False if not found.
        """
        with self.vehicles_lock:
            if vehicle_id not in self.vehicles:
                return False

            removed_vehicle = self.vehicles.pop(vehicle_id)
            self.metrics.total_vehicles = len(self.vehicles)

            self.emit_event(
                "vehicle_removed",
                {
                    "vehicle_id": vehicle_id,
                    "vehicle": removed_vehicle,
                    "total_vehicles": self.metrics.total_vehicles,
                },
            )

            return True

    def get_vehicles(self) -> list[Vehicle]:
        """Get a thread-safe copy of all vehicles.

        Returns:
            List of all vehicles in the simulation.
        """
        with self.vehicles_lock:
            return list(self.vehicles.values())

    def run(self):
        """Start the simulation in a separate thread."""
        if self.running:
            return

        self.running = True
        self.pause_event.set()  # Initially not paused

        # Start main simulation thread
        self.main_thread = threading.Thread(
            target=self._simulation_loop, name="SimulationMain", daemon=False
        )
        self.main_thread.start()

        # Emit start event
        self.emit_event(
            "simulation_started", {"timestamp": time.time(), "config": self.config}
        )

    def stop(self):
        """Stop the simulation gracefully."""
        if not self.running:
            return

        self.running = False
        self.shutdown_event.set()

        # Wait for main thread to finish
        if self.main_thread and self.main_thread.is_alive():
            self.main_thread.join(timeout=5.0)

        # Shutdown thread pool
        if self.executor:
            self.executor.shutdown(wait=True)

        # Stop metrics collection
        if self.metrics_thread and self.metrics_thread.is_alive():
            self.metrics_thread.join(timeout=2.0)

        # Emit stop event
        self.emit_event(
            "simulation_stopped",
            {"timestamp": time.time(), "final_metrics": self.metrics},
        )

    def pause(self):
        """Pause the simulation."""
        self.paused = True
        self.pause_event.clear()
        self.emit_event("simulation_paused", {"timestamp": time.time()})

    def resume(self):
        """Resume the simulation."""
        self.paused = False
        self.pause_event.set()
        self.emit_event("simulation_resumed", {"timestamp": time.time()})

    def _simulation_loop(self):
        """Main simulation loop running in separate thread."""
        last_update = time.time()

        while self.running and not self.shutdown_event.is_set():
            # Handle pause
            self.pause_event.wait()

            if not self.running:
                break

            current_time = time.time()
            real_dt = current_time - last_update

            # Control simulation speed
            if self.config.enable_real_time:
                if real_dt < self.config.update_interval:
                    time.sleep(self.config.update_interval - real_dt)
                    current_time = time.time()
                    real_dt = current_time - last_update

            # Update simulation
            sim_dt = Second(real_dt)
            update_start = time.time()

            try:
                self.sim_update(sim_dt)
                self.sim_post_update(sim_dt)

                # Update timing
                self.simulation_time += float(sim_dt)

            except Exception as e:
                self.emit_event(
                    "simulation_error", {"error": str(e), "timestamp": time.time()}
                )
                break

            # Record performance
            update_time = time.time() - update_start
            self.update_times.append(update_time)

            # Limit update times history
            if len(self.update_times) > 100:
                self.update_times = self.update_times[-50:]

            last_update = current_time

    def sim_update(self, dt: Time):
        """Update all vehicles using thread pool."""
        vehicles = self.get_vehicles()

        if not vehicles:
            return

        # Split vehicles into batches for processing
        batches = [
            vehicles[i : i + self.config.batch_size]
            for i in range(0, len(vehicles), self.config.batch_size)
        ]

        # Process batches in parallel
        futures = []
        current_time = Time(self.simulation_time)

        for batch in batches:
            future = self.executor.submit(
                self._update_vehicle_batch, batch, dt, current_time
            )
            futures.append(future)

        # Wait for all batches to complete
        for future in as_completed(futures):
            try:
                future.result()  # This will raise any exceptions that occurred
            except Exception as e:
                self.emit_event(
                    "batch_update_error", {"error": str(e), "timestamp": time.time()}
                )

    def _update_vehicle_batch(self, vehicles: list[Vehicle], dt: Time, now: Time):
        """Update a batch of vehicles in a worker thread."""
        for vehicle in vehicles:
            try:
                vehicle.update(dt, now)
            except Exception as e:
                self.emit_event(
                    "vehicle_update_error",
                    {
                        "vehicle_id": vehicle.id,
                        "error": str(e),
                        "timestamp": time.time(),
                    },
                )

    def sim_post_update(self, dt: Time):
        """Post-update processing using thread pool."""
        vehicles = self.get_vehicles()
        current_time = Time(self.simulation_time)

        # Process post-updates in parallel
        futures = []
        for vehicle in vehicles:
            future = self.executor.submit(
                self._post_update_vehicle, vehicle, dt, current_time
            )
            futures.append(future)

        # Wait for completion
        for future in as_completed(futures):
            try:
                future.result()
            except Exception as e:
                self.emit_event(
                    "post_update_error", {"error": str(e), "timestamp": time.time()}
                )

    def _post_update_vehicle(self, vehicle: Vehicle, dt: Time, now: Time):
        """Post-update processing for a single vehicle."""
        try:
            vehicle.post_update(dt, now)
        except Exception as e:
            self.emit_event(
                "vehicle_post_update_error",
                {"vehicle_id": vehicle.id, "error": str(e), "timestamp": time.time()},
            )

    def _metrics_collector(self):
        """Background thread for collecting performance metrics."""
        while self.running and not self.shutdown_event.is_set():
            try:
                current_time = time.time()

                # Update metrics
                self.metrics.simulation_time = self.simulation_time
                self.metrics.real_time = current_time - self.start_real_time

                if self.metrics.real_time > 0:
                    self.metrics.time_ratio = (
                        self.metrics.simulation_time / self.metrics.real_time
                    )

                if self.update_times:
                    self.metrics.average_update_time = sum(self.update_times) / len(
                        self.update_times
                    )
                    self.metrics.updates_per_second = (
                        1.0 / self.metrics.average_update_time
                        if self.metrics.average_update_time > 0
                        else 0
                    )

                self.metrics.active_threads = threading.active_count()

                # Emit metrics event
                if current_time - self.last_metrics_update >= 1.0:  # Every second
                    self.emit_event(
                        "metrics_updated",
                        {"metrics": self.metrics, "timestamp": current_time},
                    )
                    self.last_metrics_update = current_time

                time.sleep(0.5)  # Update metrics twice per second

            except Exception as e:
                self.emit_event(
                    "metrics_error", {"error": str(e), "timestamp": time.time()}
                )

    # Event System
    def register_event_handler(self, event_name: str, handler: Callable):
        """Register an event handler for the specified event.

        Args:
            event_name: Name of the event to handle.
            handler: Callable to handle the event.
        """
        if event_name not in self.event_handlers:
            self.event_handlers[event_name] = []
        self.event_handlers[event_name].append(handler)

    def emit_event(self, event_name: str, data: Any):
        """Emit an event to all registered handlers.

        Args:
            event_name: Name of the event to emit.
            data: Data to pass to event handlers.
        """
        if event_name in self.event_handlers:
            for handler in self.event_handlers[event_name]:
                try:
                    # Run handler in thread pool to avoid blocking
                    if self.executor:
                        self.executor.submit(handler, event_name, data)
                except Exception as e:
                    print(f"Error in event handler for {event_name}: {e}")

    def results(self) -> dict[str, Any]:
        """Get simulation results and metrics.

        Returns:
            Dictionary containing simulation results and performance metrics.
        """
        return {
            "metrics": self.metrics,
            "vehicles": {
                vid: (
                    vehicle.get_status()
                    if hasattr(vehicle, "get_status")
                    else str(vehicle)
                )
                for vid, vehicle in self.vehicles.items()
            },
            "configuration": self.config,
            "events_processed": len(self.event_handlers),
            "simulation_complete": not self.running,
        }


# Example usage and specialized simulator
class DroneFleetSimulator(ThreadedSimulator):
    """Specialized simulator for drone fleet operations."""

    def __init__(self, num_drones: int = 10, config: SimulationConfig | None = None):
        super().__init__(config)
        self.num_drones = num_drones

        # Register event handlers
        self.register_event_handler("vehicle_added", self._on_vehicle_added)
        self.register_event_handler("metrics_updated", self._on_metrics_updated)

    def init_vehicles(self):
        """Initialize drone fleet."""
        from dronesim.energy.battery import BatteryStatus
        from dronesim.energy.unit import WattHour
        from dronesim.geo import GeoPoint
        from dronesim.unit import KilometersPerHour, Watt
        from dronesim.vehicles import Drone

        # Create drones at different positions
        for i in range(self.num_drones):
            pos = GeoPoint.from_deg(37.5665 + i * 0.01, 126.9780 + i * 0.01)
            battery = BatteryStatus(WattHour(100.0), WattHour(80.0))

            drone = Drone(
                pos=pos,
                battery=battery,
                velocity=KilometersPerHour(50.0),
                power_transit=Watt(25.0),
            )

            self.add_vehicle(drone)

    def _on_vehicle_added(self, event_name: str, data: dict[str, Any]):
        """Handle vehicle added event."""
        print(f"드론 {data['vehicle_id']} 추가됨. 총 {data['total_vehicles']}대")

    def _on_metrics_updated(self, event_name: str, data: dict[str, Any]):
        """Handle metrics updated event."""
        metrics = data["metrics"]
        print(
            f"성능 지표 - UPS: {metrics.updates_per_second:.1f}, "
            f"평균 업데이트 시간: {metrics.average_update_time*1000:.1f}ms, "
            f"활성 스레드: {metrics.active_threads}"
        )


# Usage example
if __name__ == "__main__":
    # Create configuration
    config = SimulationConfig(
        max_workers=6,
        update_interval=0.05,  # 20 FPS
        batch_size=5,
        enable_real_time=True,
        log_performance=True,
    )

    # Create and run simulation
    sim = DroneFleetSimulator(num_drones=20, config=config)

    try:
        sim.init_data()
        sim.init_vehicles()
        sim.run()

        # Run for 30 seconds
        time.sleep(30)

        # Get results
        results = sim.results()
        print("시뮬레이션 결과:", results["metrics"])

    finally:
        sim.stop()
