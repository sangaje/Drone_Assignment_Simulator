from abc import ABC, abstractmethod
from dronesim.unit import Time, Second
from dronesim.vehicles import Vehicle
from dronesim.mission import Task
from queue import Queue
from concurrent.futures import ThreadPoolExecutor, as_completed
from rich.progress import (Progress, SpinnerColumn, TextColumn, BarColumn, TimeElapsedColumn, TimeRemainingColumn, TaskProgressColumn, track)
from typing import List


class Simulator[V: Vehicle, T: Task](ABC):

    _j: int
    _batch_size: int
    _dt: Time
    _vehicles: List[V]
    _tasks_queue: Queue[T]
    _pending_tasks_queue: Queue[T]
    _working_tasks_queue: Queue[T]
    _completed_tasks_queue: Queue[T]
    _executor: ThreadPoolExecutor

    def __init__(self, j: int = 1, batch_size: int = 1, dt: Time = Second(1)):
        self._j = j
        self._batch_size = batch_size
        self._dt = dt

        self._init_queues()

    @property
    def j(self) -> int:
        return self._j
    
    @property
    def batch_size(self) -> int:
        return self._batch_size
    
    @property
    def dt(self) -> Time:
        return self._dt

    def _init_queues(self):
        # Initialize Queues for tasks
        self._tasks_queue = Queue()
        self._pending_tasks_queue = Queue()
        self._working_tasks_queue = Queue()
        self._completed_tasks_queue = Queue()

    def _init_thread_pool_executor(self):
        self._executor = ThreadPoolExecutor(max_workers=self.j, thread_name_prefix="SimulatorWorker")

    def run(self):
        with Progress(
            TextColumn("[progress.description]{task.description}"),
            SpinnerColumn(),
            BarColumn(bar_width=40),
            TaskProgressColumn(),          # 예: 123/1000 • 12.3%
            TimeElapsedColumn(),
            TimeRemainingColumn(),
        ) as progress:
            self._init_sim(progress)
            

            try:
                self._sim_loop()
            
            except Exception as e:
                self._shutdown_executor(hard_shutdown=True)
                raise e

            finally:
                self._shutdown_executor()

    def _init_sim(self, progress: Progress):
        init_tasks = [
            self._init_queues,
            self.init_data,
            self.init_vehicles,
            self._init_thread_pool_executor

        ]
        init_msg = "[green]Initializing Simulation..."
        init_progress = progress.add_task(init_msg, total=len(init_tasks))
        for init_task in init_tasks:
            init_task(progress)
            progress.advance(init_progress)


    def _sim_loop(self):
        while not self.done:
            # Updates (in Vehicle methods)
            # Vehile updates
            # Simulation updates
            # Post updates (in Vehicle methods)
            # Simulation post updates


    @abstractmethod
    def results(self):
        pass


    @abstractmethod
    def init_data(self, progress: Progress):
        """Initialize simulation data and state.

        This method should be implemented by subclasses to define how the
        simulation data and initial state are set up before running the simulation.
        """
        pass

    @abstractmethod
    def init_vehicles(self, progress: Progress):
        """Initialize vehicles in the simulation.

        This method should be implemented by subclasses to define how the
        vehicles are created and configured for the simulation.
        """
        pass

    @abstractmethod
    def sim_update(self, dt):
        """Update the simulation state by a time step.

        This method should be implemented by subclasses to define how the
        simulation state is updated for each time step.

        Args:
            dt: Time step duration for this update cycle.
        """
        pass

    @abstractmethod
    def sim_post_update(self, dt):
        """Perform any necessary operations after the simulation update.

        This method should be implemented by subclasses to define any
        post-update operations that need to be performed after the main
        simulation update.

        Args:
            dt: Time step duration for this update cycle.
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
        
        is_all_tasks_pending = self._tasks_queue.empty()
        is_all_tasks_working_out = self._pending_tasks_queue.empty()
        is_all_tasks_completed = self._working_tasks_queue.empty()
        return is_all_tasks_pending and is_all_tasks_working_out and is_all_tasks_completed

    def _shutdown_executor(self, hard_shutdown: bool = False):
        self._executor.shutdown(wait=True, cancel_futures=hard_shutdown)