"""Abstract simulation framework for drone fleet operations.

This module provides the core simulation infrastructure for managing drone fleets
and task execution with multi-threaded processing capabilities. It implements
a generic, extensible simulation framework that supports various vehicle types
and mission configurations through abstract base classes and protocols.

Key Features:
    • Generic type system supporting custom Vehicle and Task implementations
    • Multi-threaded execution with configurable parallelism and batch processing
    • Task lifecycle management with priority queuing and state tracking
    • Rich progress visualization with real-time simulation monitoring
    • CSV-based data loading with customizable parsing and sorting
    • Thread-safe operations with proper resource management and cleanup

Architecture Overview:
    The simulation follows a producer-consumer pattern where tasks are loaded
    from external data sources and distributed among vehicles for execution.
    The framework manages the complete lifecycle from task queuing through
    completion tracking with concurrent processing capabilities.

    Simulation Flow:
        1. Data Loading: Parse CSV datasets into task objects
        2. Vehicle Initialization: Create and configure vehicle fleet
        3. Task Distribution: Assign tasks to vehicles based on availability
        4. Parallel Execution: Update vehicles and tasks concurrently
        5. Progress Tracking: Monitor completion and performance metrics
        6. Resource Cleanup: Properly shutdown threads and free resources

Threading Model:
    • Main Thread: Simulation control loop and coordination
    • Worker Threads: Parallel vehicle updates and task processing
    • Progress Thread: Real-time UI updates and monitoring (Rich library)

Usage Pattern:
    >>> class MySimulator(Simulator[MyVehicle, MyTask]):
    ...     def make_task(self, columns, row):
    ...         return MyTask.from_csv_row(columns, row)
    ...
    ...     def make_vehicle(self):
    ...         for i in range(10):
    ...             yield MyVehicle(id=i)
    ...
    ...     def sim_update(self, dt, now):
    ...         # Custom simulation logic
    ...         self._assign_pending_tasks()
    >>> sim = MySimulator()
    >>> sim.run("data.csv", key=lambda task: task.priority)
"""

from abc import ABC, abstractmethod
from collections import deque
from collections.abc import Callable, Iterator, Sequence
from concurrent.futures import ALL_COMPLETED, ThreadPoolExecutor, wait
import csv
from enum import Enum
from typing import Generic, Protocol, Self, TypeVar, runtime_checkable

import numpy as np
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.progress import (
    BarColumn,
    Progress,
    SpinnerColumn,
    TaskProgressColumn,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
)
from rich.table import Table
from rich.text import Text

from dronesim.mission import Task
from dronesim.state.state_machine import State
from dronesim.unit import Second, Time
from dronesim.unit.unit_time import ClockTime
from dronesim.vehicles import Vehicle

ONE_SECOND = Second(1)
CONSOLE = Console()
V = TypeVar("V", bound=Vehicle)
T = TypeVar("T", bound=Task)


@runtime_checkable
class _SupportsRichComparisonT(Protocol):
    """Protocol for types that support rich comparison operations.

    This protocol defines the interface for types that can be compared using
    all standard comparison operators (<, <=, >, >=, ==, !=). It is used
    as a type constraint for sorting operations in the simulation framework.

    The protocol is runtime-checkable, meaning isinstance() checks can be
    performed to verify if an object supports these comparison operations.

    Methods:
        All comparison methods follow Python's standard comparison protocol:
        - __lt__: Less than comparison (self < other)
        - __le__: Less than or equal comparison (self <= other)
        - __gt__: Greater than comparison (self > other)
        - __ge__: Greater than or equal comparison (self >= other)
        - __eq__: Equality comparison (self == other)
        - __ne__: Inequality comparison (self != other)

    Usage:
        This protocol is primarily used in the simulator's task sorting
        functionality where tasks need to be ordered by priority, time,
        or other comparable attributes.

    Example:
        >>> @dataclass
        ... class PriorityTask:
        ...     priority: int
        ...
        ...     def __lt__(self, other):
        ...         return self.priority < other.priority
        ...
        ...     # ... other comparison methods
        >>> isinstance(PriorityTask(1), _SupportsRichComparisonT)
        True
    """

    def __lt__(self, other: Self, /) -> bool: ...
    def __le__(self, other: Self, /) -> bool: ...
    def __gt__(self, other: Self, /) -> bool: ...
    def __ge__(self, other: Self, /) -> bool: ...
    def __eq__(self, other: object, /) -> bool: ...
    def __ne__(self, other: object, /) -> bool: ...


class Simulator(ABC, Generic[V, T]):
    """Abstract base class for drone fleet simulation with multi-threaded execution.

    This generic simulation framework manages the complete lifecycle of vehicle-based
    task execution with concurrent processing capabilities. It provides infrastructure
    for loading tasks from CSV data, managing vehicle fleets, and orchestrating
    parallel simulation updates with progress monitoring.

    The simulator follows a template method pattern where subclasses implement
    specific behaviors for task creation, vehicle initialization, and simulation
    logic while the base class handles threading, progress tracking, and resource
    management.

    Type Parameters:
        V: Vehicle type that extends the Vehicle base class
        T: Task type that extends the Task base class

    Architecture:
        The simulation operates through distinct phases managed by separate queues:

        Task Lifecycle Queues:
        • _tasks_queue: Initial tasks loaded from data source
        • _pending_tasks_queue: Tasks ready for assignment to vehicles
        • _working_tasks_queue: Tasks currently being executed by vehicles
        • _completed_tasks_queue: Successfully completed tasks

        Execution Model:
        • Main thread controls simulation loop and coordination
        • ThreadPoolExecutor manages parallel vehicle updates
        • Rich Progress provides real-time visualization
        • Batch processing optimizes performance for large fleets

    Key Features:
        • Generic type system for custom vehicle and task implementations
        • CSV data loading with configurable parsing and sorting
        • Multi-threaded execution with configurable batch sizes
        • Real-time progress monitoring with rich terminal UI
        • Automatic resource management and cleanup
        • Extensible update hooks for custom simulation logic

    Attributes:
        _vehicles (list[V]): Fleet of vehicles participating in simulation
        _tasks_queue (deque[T]): Queue of tasks awaiting processing
        _pending_tasks_queue (deque[T]): Tasks ready for vehicle assignment
        _working_tasks_queue (deque[T]): Tasks currently being executed
        _completed_tasks_queue (deque[T]): Successfully completed tasks
        _executor (ThreadPoolExecutor): Thread pool for parallel processing

    Abstract Methods:
        Subclasses must implement these methods for specific simulation behavior:
        • make_task(columns, row): Create task objects from CSV data
        • make_vehicle(): Generate vehicle instances for the fleet
        • sim_update(dt, now): Custom simulation logic per time step
        • sim_post_update(dt, now): Post-update processing per time step
        • results(): Generate final simulation results and metrics

    Example Implementation:
        >>> class DroneDeliverySimulator(Simulator[DeliveryDrone, DeliveryTask]):
        ...     def make_task(self, columns, row):
        ...         return DeliveryTask.from_csv(columns, row)
        ...
        ...     def make_vehicle(self):
        ...         for i in range(self.fleet_size):
        ...             yield DeliveryDrone(id=i, base_location=self.base)
        ...
        ...     def sim_update(self, dt, now):
        ...         self._assign_tasks_to_available_drones()
        ...         self._update_delivery_metrics()
        >>> sim = DroneDeliverySimulator()
        >>> sim.run("deliveries.csv", key=lambda t: t.priority_score)
    """

    _vehicles: list[V]
    _tasks_queue: deque[T]
    _pending_tasks_queue: deque[T]
    _working_tasks_queue: deque[T]
    _completed_tasks_queue: deque[T]
    _cooldown_tasks_queue: deque[T]
    _temp_cooldown_tasks_queue: list[T]
    _temp_working_tasks_queue: list[T]
    _executor: ThreadPoolExecutor

    _temp_task_texts: dict[State, Text]
    _temp_vehicle_texts: dict[State, Text]
    _task_time_mean: Text
    _task_time_std: Text
    _max_vehicle_utilization: Text

    _dt: Time


    def __init__(self):
        """Initialize a new Simulator instance.

        Sets up the basic simulation infrastructure including empty vehicle list
        and task queues. The thread pool executor is initialized later during
        the run() method to allow for configuration of worker threads.

        Note:
            This constructor only performs basic initialization. The full simulation
            setup occurs in the run() method which handles data loading, vehicle
            creation, and thread pool configuration.
        """
        self._vehicles = []
        self._executor = None

        self._init_queues()

    def _init_queues(self):
        # Initialize deques for tasks
        self._tasks_queue = deque()
        self._pending_tasks_queue = deque()
        self._working_tasks_queue = deque()
        self._completed_tasks_queue = deque()
        self._cooldown_tasks_queue = deque()
        self._temp_cooldown_tasks_queue = []
        self._temp_working_tasks_queue = []

    def _init_thread_pool_executor(self, progress: Progress, j: int):
        init_msg = "[green]Initializing Thread Pool Executor..."
        p = progress.add_task(init_msg, total=1)
        self._executor = ThreadPoolExecutor(j, "SimulatorWorker")
        progress.advance(p)

    def parse_task_data(
        self,
        dataset_path: str,
        key: Callable[[T], _SupportsRichComparisonT] | None,
        progress: Progress | None = None,
    ) -> Iterator[T]:
        with open(dataset_path, encoding="utf-8") as f:
            reader = csv.reader(f)
            columns = next(reader)

            init_msg = "[green]Loading Simulation Data..."
            if progress:
                p = progress.add_task(init_msg, total=sum(1 for _ in f) + 1)
            f.seek(0)
            next(reader)  # 헤더 건너뛰기
            temp_tasks_list: list[T] = []
            for row in reader:
                row = [cell.strip() for cell in row]
                task = self.make_task(columns, row)
                if progress:
                    progress.advance(p)
                if task is not None:
                    temp_tasks_list.append(task)

            if key is None:
                for task in temp_tasks_list:
                    yield task
            else:
                for task in sorted(temp_tasks_list, key=key):
                    yield task
            if progress:
                progress.advance(p)

    def _init_data(
        self,
        progress: Progress,
        dataset_path: str,
        key: Callable[[T], _SupportsRichComparisonT] | None,
    ):
        """Initialize simulation data and state.

        This method should be implemented by subclasses to define how the simulation data and
        initial state are set up before running the simulation.
        """
        for task in self.parse_task_data(dataset_path, key, progress):
            self._tasks_queue.append(task)

    def _init_vehicles(self, progress: Progress):
        init_msg = "[green]Initializing Vehicles..."
        p = progress.add_task(init_msg, total=None)
        count = 0
        for vehicle_or_vehicles in self.make_vehicle():
            if isinstance(vehicle_or_vehicles, Sequence):
                self._vehicles.extend(vehicle_or_vehicles)
                vehicles_count = len(vehicle_or_vehicles)
            else:
                self._vehicles.append(vehicle_or_vehicles)
                vehicles_count = 1
            progress.advance(p, vehicles_count)
            count += vehicles_count
        progress.update(p, total=count, completed=count)

    def run(
        self,
        dataset_path: str,
        key: Callable[[T], _SupportsRichComparisonT] | None,
        j: int = 1,
        batch_size: int = 1,
        dt: Time = ONE_SECOND,
    ):
        """Execute the complete simulation from initialization through completion.

        This is the main entry point for running a simulation. It orchestrates the
        entire simulation lifecycle including data loading, vehicle initialization,
        thread pool setup, and the main simulation loop with progress monitoring.

        The method handles all resource management automatically, including proper
        cleanup of threads and executors even if exceptions occur during execution.

        Args:
            dataset_path (str): Path to the CSV file containing task data.
                               The file must have a header row followed by data rows.
            key (Callable[[T], _SupportsRichComparisonT] | None): Optional sorting
                function for tasks. If provided, tasks will be sorted before
                processing. If None, tasks are processed in file order.
            j (int): Number of worker threads in the thread pool executor.
                    Defaults to 1 for single-threaded execution.
            batch_size (int): Number of vehicles to process in each parallel batch.
                             Larger batches can improve performance but use more memory.
                             Defaults to 1.
            dt (Time): Time step duration for each simulation update cycle.
                      Defaults to 1 second.

        Raises:
            FileNotFoundError: If the dataset_path file cannot be found.
            ValueError: If the CSV data cannot be parsed or contains invalid values.
            RuntimeError: If thread pool initialization or vehicle creation fails.

        Note:
            This method blocks until the simulation completes or an error occurs.
            Progress is displayed using Rich progress bars showing task queues,
            vehicle status, and elapsed/remaining time estimates.

        Example:
            >>> simulator = MySimulator()
            >>> simulator.run(
            ...     dataset_path="tasks.csv",
            ...     key=lambda task: task.priority,
            ...     j=4,  # Use 4 worker threads
            ...     batch_size=10,  # Process 10 vehicles per batch
            ...     dt=Second(0.1),  # 100ms time steps
            ... )
        """
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            TextColumn("• Run: {task.completed} times"),
            BarColumn(bar_width=40),
            TaskProgressColumn(),  # 예: 123/1000 • 12.3%
            TimeElapsedColumn(),
            TimeRemainingColumn(),
            console=CONSOLE,
            auto_refresh=True,
        ) as progress:
            panel = self._init_sim(progress, dataset_path, key, j, dt)
            try:
                self._sim_loop(progress, batch_size, dt, key, panel)
            except Exception:
                self._shutdown_executor(hard_shutdown=True)
                raise
            finally:
                self._shutdown_executor()

    def _init_sim(
        self,
        progress: Progress,
        dataset_path: str,
        key: Callable[[T], _SupportsRichComparisonT] | None,
        j: int,
        dt: Time
    ) -> Panel:
        init_msg = "[green]Initializing Simulation..."
        p = progress.add_task(init_msg, total=4)
        self._dt = dt
        self._init_queues()
        self._temp_task_texts = {}
        self._temp_vehicle_texts = {}
        self._init_data(progress, dataset_path, key)
        progress.advance(p)
        self._init_vehicles(progress)
        progress.advance(p)
        self._init_thread_pool_executor(progress, j)
        progress.advance(p)

        tasks_states: list[Enum] = self._tasks_queue[0].state_list()
        vehicles_states: list[Enum] = self._vehicles[0].state_list()
        self._task_time_mean = Text("None")
        self._task_time_std = Text("None")
        self._max_vehicle_utilization = Text("0%")
        self._operational_vehicles = Text("0%")

        self._pending_queue = Text("0")
        self._cooldown_queue = Text("0")
        self._working_queue = Text("0")
        self._completed_queue = Text("0")

        self._current_max_vehicle_utilization = 0.0

        t = Table.grid(padding=(0, 2))
        for state in tasks_states:
            self._temp_task_texts[state] = Text("0")
            t.add_row(f"[b]Task - {state.name}[/b]: ", self._temp_task_texts[state])

        for state in vehicles_states:
            self._temp_vehicle_texts[state] = Text("0")
            t.add_row(
                f"[b]Vehicle - {state.name}[/b]: ", self._temp_vehicle_texts[state]
            )

        t.add_section()
        t.add_row("[b]Average Task Time[/b]: ", self._task_time_mean)
        t.add_row("[b]Task Time Standard Deviation[/b]: ", self._task_time_std)
        t.add_row("[b]Max Vehicle Utilization[/b]: ", self._max_vehicle_utilization)
        t.add_row("[b]Operational Vehicles Percentage[/b]: ", self._operational_vehicles)

        t.add_section()
        t.add_row("[b]Pending Queue Size[/b]: ", self._pending_queue)
        t.add_row("[b]Cooldown Queue Size[/b]: ", self._cooldown_queue)
        t.add_row("[b]Working Queue Size[/b]: ", self._working_queue)
        t.add_row("[b]Completed Queue Size[/b]: ", self._completed_queue)


        progress.advance(p)

        return Panel(t, title="Current States", padding=(1, 2))

    def _sim_loop(
        self,
        progress: Progress,
        batch_size: int,
        dt: Time,
        key: Callable[[T], _SupportsRichComparisonT] | None,
        panel: Panel = None,
    ):
        now = ClockTime(0)
        t = progress.add_task("", total=None)
        tasks_count = len(self._tasks_queue)
        self._progress = progress
        self._t_waiting = progress.add_task(
            "[green]Pending Tasks...", total=tasks_count
        )
        self._t_working = progress.add_task(
            "[green]Working Tasks...", total=tasks_count
        )
        self._t_completed = progress.add_task(
            "[green]Completed Tasks...", total=tasks_count
        )

        def execute_parallel(fn: Callable, batch_size: int, *args):
            if self._executor._max_workers == 1:
                for i in range(0, len(self._vehicles), batch_size):
                    fn(i, min(i + batch_size, len(self._vehicles)), *args)
                return
            futures = [
                self._executor.submit(
                    fn, i, min(i + batch_size, len(self._vehicles)), *args
                )
                for i in range(0, len(self._vehicles), batch_size)
            ]
            done, _ = wait(list(futures), return_when=ALL_COMPLETED)
            for future in done:
                future.result()

        def do_update(start: int, end: int, dt: Time, now: Time):
            try:
                for i in range(start, end):
                    self._vehicles[i].update(dt, now)
            except Exception as e:
                print(f"Error during vehicle update in range ({start}, {end}): {e}")

        def do_vehicle_update(start: int, end: int, dt: Time, now: Time):
            try:
                for i in range(start, end):
                    self._vehicles[i].vehicle_update(dt, now)
            except Exception as e:
                print(f"Error during vehicle update in range ({start}, {end}): {e}")

        def do_refresh_timer(start: int, end: int, dt: Time, now: Time):
            try:
                for i in range(start, end):
                    self._vehicles[i].timer_update(dt)
            except Exception as e:
                print(f"Error during vehicle update in range ({start}, {end}): {e}")

        def do_post_update(start: int, end: int, dt: Time, now: Time):
            try:
                for i in range(start, end):
                    self._vehicles[i].post_update(dt, now)
            except Exception as e:
                print(f"Error during vehicle update in range ({start}, {end}): {e}")

        def do_task_update():
            if key is None:
                while len(self._tasks_queue) > 0:
                    task = self._tasks_queue.popleft()
                    self._pending_tasks_queue.append(task)
                    progress.advance(self.self._t_waiting)
            else:
                while len(self._tasks_queue) > 0 and key(self._tasks_queue[0]) <= now:
                    task = self._tasks_queue.popleft()
                    task.start_at = now
                    self._pending_tasks_queue.append(task)
                    progress.advance(self._t_waiting)

            # TODO How to improve efficiency?
            for i in range(len(self._working_tasks_queue)):
                task = self._working_tasks_queue[i]
                if task.done:
                    task.completed_at = now

            while (
                len(self._working_tasks_queue) > 0 and self._working_tasks_queue[0].done
            ):
                task = self._working_tasks_queue.popleft()
                self._completed_tasks_queue.append(task)
                progress.advance(self._t_completed)

        def refresh_state_counts():
            # Update task state counts
            task_state_counts = dict.fromkeys(self._temp_task_texts.keys(), 0)
            for task in (
                list(self._pending_tasks_queue)
                + list(self._working_tasks_queue)
                + list(self._completed_tasks_queue)
            ):
                task_state_counts[task.current_state] += 1
            for state, text in self._temp_task_texts.items():
                text.plain = str(task_state_counts[state])

            # Update vehicle state counts
            vehicle_state_counts = dict.fromkeys(self._temp_vehicle_texts.keys(), 0)

            vehicle_usage = 0
            vehicle_oprational = 0
            for vehicle in self._vehicles:
                vehicle_state_counts[vehicle.current_state] += 1
                if vehicle.is_busy:
                    vehicle_usage += 1
                if vehicle.is_operational():
                    vehicle_oprational += 1

            for state, text in self._temp_vehicle_texts.items():
                text.plain = str(vehicle_state_counts[state])

            if len(self._completed_tasks_queue) == 0:
                self._task_time_mean.plain = "--:--:--"
                self._task_time_std.plain = "--:--:--"

            else:
                start_times = np.asarray(
                    [float(task.start_at) for task in self._completed_tasks_queue]
                )
                end_times = np.asarray(
                    [float(task.completed_at) for task in self._completed_tasks_queue]
                )

                d_times = end_times - start_times
                mean = ClockTime(np.mean(d_times))
                std = ClockTime(np.std(d_times))
                self._task_time_mean.plain = str(mean)
                self._task_time_std.plain = str(std)

            vehicle_utilization = vehicle_usage / len(self._vehicles) * 100


            self._current_max_vehicle_utilization = max(
                self._current_max_vehicle_utilization, vehicle_utilization
            )
            self._max_vehicle_utilization.plain = f"{self._current_max_vehicle_utilization:.1f}%"


            opreational_perventage = vehicle_oprational / len(self._vehicles) * 100
            self._operational_vehicles.plain = f"{opreational_perventage:.1f}%"

            self._pending_queue.plain = str(len(self._pending_tasks_queue))
            self._cooldown_queue.plain = str(len(self._cooldown_tasks_queue))
            self._working_queue.plain = str(len(self._working_tasks_queue))
            self._completed_queue.plain = str(len(self._completed_tasks_queue))

        def reassign_cooldown_tasks():
            self._temp_cooldown_tasks_queue.sort(key= lambda t: t.start_at)

            for task in self._temp_cooldown_tasks_queue:
                self._cooldown_tasks_queue.append(task)
                self._progress.advance(self._t_working, -1)

            self._temp_cooldown_tasks_queue.clear()

            for task in self._temp_working_tasks_queue:
                self._working_tasks_queue.append(task)

            self._temp_working_tasks_queue.clear()

        with Live(panel, console=CONSOLE, auto_refresh=True) as _:
            while not self.done:
                execute_parallel(do_update, batch_size, dt, now)
                execute_parallel(do_refresh_timer, batch_size, dt, now)
                execute_parallel(do_vehicle_update, batch_size, dt, now) # recharge battery -> enter ground
                self.sim_update(dt, now)
                execute_parallel(do_post_update, batch_size, dt, now)
                self.sim_post_update(dt, now)
                do_task_update()
                progress.update(t, description=f"[green]Simulation Time: {now}")
                reassign_cooldown_tasks()
                refresh_state_counts()

                now += dt

    def get_pending_tasks(self) -> Iterator[T]:
        """Get a list of all pending tasks in the simulation.

        Returns:
            Sequence[T]: A sequence of pending tasks currently in the simulation.
        """
        while len(self._cooldown_tasks_queue) > 0:
            task = self._cooldown_tasks_queue.popleft()
            self._temp_working_tasks_queue.append(task)
            # self._working_tasks_queue.append(task)
            self._progress.advance(self._t_working)
            yield task

        while len(self._pending_tasks_queue) > 0:
            self._progress.advance(self._t_working)
            task = self._pending_tasks_queue.popleft()
            self._temp_working_tasks_queue.append(task)
            # self._working_tasks_queue.append(task)
            yield task



    def failed_to_assign_task(self, task: T, dt: Time = None):
        self._temp_working_tasks_queue.remove(task)
        if dt is not None:
            task.priority += (0.05/60) * float(dt)
        self._temp_cooldown_tasks_queue.append(task)


    def get_tasks(self) -> Sequence[T]:
        return list(self._tasks_queue)

    def get_vehicles(self) -> Sequence[V]:
        """Get a list of all vehicles in the simulation.

        Returns:
            Sequence[V]: A sequence of vehicles currently in the simulation.
        """
        return self._vehicles

    @abstractmethod
    def make_task(self, columns: list[str], row: list[str]) -> T | None:
        """Create a task instance from CSV data row.

        This abstract method must be implemented by subclasses to define how
        CSV data rows are parsed and converted into task objects. The method
        receives the column headers and a data row, and should return a task
        instance or None if the row should be skipped.

        Args:
            columns (list[str]): List of column headers from the CSV file.
                               These can be used to map row data to task attributes.
            row (list[str]): List of data values corresponding to the columns.
                            Values are strings and may need type conversion.

        Returns:
            T | None: A task instance of type T if the row contains valid data,
                     or None if the row should be skipped or cannot be parsed.

        Example:
            >>> def make_task(self, columns, row):
            ...     if len(row) < 3:
            ...         return None  # Skip incomplete rows
            ...
            ...     col_map = {col: i for i, col in enumerate(columns)}
            ...     return DeliveryTask(
            ...         origin=GeoPoint.from_str(row[col_map["pickup"]]),
            ...         destination=GeoPoint.from_str(row[col_map["dropoff"]]),
            ...         priority=int(row[col_map["priority"]]),
            ...     )
        """
        pass

    @abstractmethod
    def make_vehicle(self) -> Iterator[V | Sequence[V]] | None:
        """Generate vehicle instances for the simulation fleet.

        This abstract method must be implemented by subclasses to create and
        configure the vehicles that will participate in the simulation. The
        method should yield either individual vehicles or sequences of vehicles.

        The iterator pattern allows for lazy vehicle creation and supports both
        individual vehicle generation and bulk vehicle creation depending on
        the implementation needs.

        Returns:
            Iterator[V | Sequence[V]]: An iterator that yields either:
                - Individual vehicle instances of type V
                - Sequences (lists/tuples) of vehicle instances

                The simulator will handle both cases automatically, adding
                individual vehicles directly and extending the fleet with
                sequences of vehicles.
            None: There are no any vehicles to create.

        Example:
            >>> def make_vehicle(self):
            ...     # Create individual vehicles
            ...     for i in range(10):
            ...         yield DeliveryDrone(
            ...             id=i,
            ...             battery=BatteryStatus(100, 80),
            ...             position=self.base_locations[i % len(self.base_locations)],
            ...         )
            ...
            ...     # Or create vehicles in batches
            ...     batch = [SurveillanceDrone(id=i) for i in range(10, 15)]
            ...     yield batch
        """
        pass

    @property
    def results(self) -> tuple[list[T], list[V]]:
        """Generate final simulation results and performance metrics.

        This abstract method must be implemented by subclasses to collect,
        process, and return the final results of the simulation. This method
        is typically called after the simulation completes to gather metrics,
        statistics, and other relevant data for analysis.

        The return type is flexible to accommodate different result formats
        depending on the specific simulation requirements. Common patterns
        include dictionaries with metrics, dataclasses with structured data,
        or custom result objects.

        Returns:
            Any: Simulation results in a format appropriate for the specific
                 simulation type. Common formats include:
                 - Dict with performance metrics and statistics
                 - Custom result dataclass or object
                 - List of completed tasks with metadata
                 - Performance reports and analysis data

        Example:
            >>> def results(self):
            ...     return {
            ...         "total_tasks": len(self._completed_tasks_queue),
            ...         "completion_rate": self._calculate_completion_rate(),
            ...         "average_delivery_time": self._calculate_avg_delivery_time(),
            ...         "vehicle_utilization": self._calculate_vehicle_utilization(),
            ...         "failed_tasks": len([t for t in self._tasks if t.failed]),
            ...         "simulation_duration": self._total_simulation_time,
            ...     }

        """
        return self._completed_tasks_queue, self._vehicles

    @abstractmethod
    def sim_update(self, dt: Time, now: Time):
        """Update the simulation state by a time step.

        This method should be implemented by subclasses to define how the
        simulation state is updated for each time step. It is called during
        the main simulation loop after vehicle updates but before post-update
        processing.

        This is the ideal place to implement custom simulation logic such as:
        - Task assignment to available vehicles
        - Global simulation state updates
        - Inter-vehicle coordination and communication
        - Environmental changes and event processing
        - Performance metric collection

        Args:
            dt (Time): Time step duration for this update cycle.
            now (Time): Current simulation time since start.

        Example:
            >>> def sim_update(self, dt, now):
            ...     # Assign pending tasks to available vehicles
            ...     self._assign_tasks_to_vehicles()
            ...
            ...     # Update global simulation state
            ...     self._update_weather_conditions(now)
            ...
            ...     # Process inter-vehicle communications
            ...     self._process_vehicle_messages()
        """
        pass

    @abstractmethod
    def sim_post_update(self, dt: Time, now: Time):
        """Perform any necessary operations after the simulation update.

        This method should be implemented by subclasses to define any
        post-update operations that need to be performed after the main
        simulation update and vehicle post-updates have completed.

        This is typically used for:
        - Cleanup operations and resource management
        - Final state validation and consistency checks
        - Logging and metrics collection
        - Event processing and notifications
        - Preparation for the next simulation step

        Args:
            dt (Time): Time step duration for this update cycle.
            now (Time): Current simulation time since start.

        Example:
            >>> def sim_post_update(self, dt, now):
            ...     # Clean up completed tasks
            ...     self._cleanup_completed_tasks()
            ...
            ...     # Log performance metrics
            ...     self._log_step_metrics(dt, now)
            ...
            ...     # Check for simulation end conditions
            ...     if self._should_terminate(now):
            ...         self._prepare_shutdown()
        """
        pass

    @property
    def done(self) -> bool:
        """Check if the simulation should end.

        This method should be implemented by subclasses to define the
        conditions under which the simulation should terminate.

        Returns:
            A boolean indicating whether the simulation should end.
        """
        is_all_tasks_pending = len(self._tasks_queue) == 0
        is_all_tasks_working_out = len(self._pending_tasks_queue) == 0
        is_all_tasks_completed = len(self._working_tasks_queue) == 0
        is_all_tasks_taken = len(self._cooldown_tasks_queue) == 0
        return (
            is_all_tasks_pending and is_all_tasks_working_out and is_all_tasks_completed and is_all_tasks_taken
        )

    def _shutdown_executor(self, hard_shutdown: bool = False):
        self._executor.shutdown(wait=True, cancel_futures=hard_shutdown)

    @property
    def pending_tasks_count(self) -> int:
        """Get the number of pending tasks in the simulation.

        Returns:
            int: The count of pending tasks.
        """
        return len(self._pending_tasks_queue) + len(self._cooldown_tasks_queue)
