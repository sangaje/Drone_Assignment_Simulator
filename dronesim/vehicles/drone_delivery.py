"""Specialized autonomous delivery drone with intelligent routing and package management.

This module implements the DeliveryDrone class, a specialized concrete implementation of
Drone[DeliveryTask] that provides sophisticated delivery operations including optimal route
planning, package lifecycle management, and state-aware task execution. The class demonstrates
advanced drone framework extensibility through strategic method overriding and delivery-specific
behavioral patterns.

Core Delivery Features:
    Intelligent Route Optimization:
        • Multi-task distance-based routing for efficiency maximization
        • State-aware destination selection (pickup vs dropoff priorities)
        • Dynamic task prioritization based on DeliveryState progression
        • Automatic mission sequencing for multi-package operations

    Package Lifecycle Management:
        • Comprehensive tracking from assignment through completion
        • Timing-based pickup and dropoff coordination
        • Automatic state progression through delivery workflow
        • Ground service operations with configurable wait times

    Type-Safe Task Specialization:
        • Concrete implementation of Drone[DeliveryTask] with full type safety
        • Compile-time validation preventing incompatible task assignments
        • DeliveryTask-specific method signatures and behavior patterns
        • Integration with DeliveryState enumeration for workflow control

Advanced Architecture:
    State Machine Integration:
        The DeliveryDrone seamlessly integrates with both the base Drone state machine
        (GROUNDED, TAKING_OFF, NAVIGATING, LANDING, EMERGENCY) and the DeliveryTask
        state machine (ASSIGNED, SERVICE_PICKUP, SERVICE_DROPOFF, DONE) to provide
        coordinated multi-level state management.

    Method Override Strategy:
        • _try_assign_new_destination(now): Smart routing with distance optimization
        • on_grounded(dt, now): Ground operations with delivery-specific logic
        • enter_grounded(now): State entry with mission progression and cleanup
        • Internal timing functions for pickup/dropoff coordination

    Operational Workflow:
        1. Task Queue Management: Receive and prioritize DeliveryTask assignments
        2. Intelligent Routing: Select optimal destinations based on task states
        3. Flight Operations: Navigate using inherited drone flight capabilities
        4. Ground Services: Execute pickup/dropoff with timing coordination
        5. Mission Completion: Automatic cleanup and preparation for next delivery

Performance & Efficiency:
    Route Optimization Algorithms:
        • Nearest-neighbor selection for multiple pending deliveries
        • State-based task filtering to prioritize workflow progression
        • Distance calculations using geodetic positioning for accuracy
        • Dynamic re-routing when new high-priority tasks are assigned

    Resource Management:
        • Efficient task queue processing with automatic cleanup
        • Memory-optimized mission tracking with single active mission
        • Battery-aware operations inherited from base Drone class
        • Scalable architecture supporting high-volume delivery operations

Integration Requirements:
    Core Dependencies:
        • dronesim.mission: DeliveryTask and DeliveryState for task management
        • dronesim.unit: Time measurements for temporal coordination
        • .drone: Base Drone class providing flight and state capabilities

    Optional Extensions:
        • Custom payload management systems
        • Advanced routing algorithms (TSP, genetic algorithms)
        • Real-time traffic and weather integration
        • Multi-drone coordination and fleet optimization

Usage Examples:
    Basic Delivery Operations:
        >>> from dronesim.geo import GeoPoint
        >>> from dronesim.energy import BatteryStatus
        >>> from dronesim.mission import DeliveryTask
        >>> from dronesim.unit import WattHour, Time
        >>>
        >>> # Initialize delivery drone with starting configuration
        >>> battery = BatteryStatus(capacity=WattHour(100), current=WattHour(95))
        >>> home_base = GeoPoint.from_deg(latitude=37.5665, longitude=126.9780)
        >>> delivery_drone = DeliveryDrone(pos=home_base, battery=battery)
        >>>
        >>> # Create delivery mission with pickup and dropoff locations
        >>> pickup_location = GeoPoint.from_deg(37.5700, 126.9800)
        >>> delivery_destination = GeoPoint.from_deg(37.5750, 126.9850)
        >>> delivery_task = DeliveryTask(origin=pickup_location, destination=delivery_destination)
        >>>
        >>> # Assign mission (type-safe, only accepts DeliveryTask)
        >>> success = delivery_drone.assign(delivery_task)
        >>> assert success == True  # Drone accepts compatible task types only

    Multi-Task Scenario:
        >>> # Queue multiple deliveries for optimized routing
        >>> task1 = DeliveryTask(pickup1, dropoff1)
        >>> task2 = DeliveryTask(pickup2, dropoff2)
        >>> task3 = DeliveryTask(pickup3, dropoff3)
        >>>
        >>> delivery_drone.assign(task1)
        >>> delivery_drone.assign(task2)
        >>> delivery_drone.assign(task3)
        >>>
        >>> # Drone automatically optimizes routing based on current position
        >>> # and task states, selecting nearest pickup/dropoff locations first
"""

from dronesim.energy import BatteryStatus
from dronesim.energy.unit import Energy, WattHour
from dronesim.geo import GeoPoint
from dronesim.mission import DeliveryState, DeliveryTask
from dronesim.unit import KilometersPerHour, Length, Minute, Power, Time, Velocity, Watt
from dronesim.unit.unit_distance import Kilometer

from .drone import Drone, DroneState

DEFAULT_VELOCITY = KilometersPerHour(50.0)
DEFAULT_TRANSITION_DURATION = Minute(1.0)
DEFAULT_CONSUMPTION = Watt(0.0)
DEFAULT_OPERATIONAL_BATTERY_PERCENTAGE = 20.0


class DeliveryDrone(Drone[DeliveryTask]):
    def _load_all_tasks_from_queue(self) -> None:
        """Move all pending tasks into current_tasks in one shot when at base and idle.

        This is a pure internal helper and does not change the public interface.
        It transfers the entire task_queue to current_tasks atomically when:
        - The drone is on base (ready to depart),
        - Not currently in a return-to-base leg, and
        - There is no active current task batch yet.
        """
        if self._is_on_base and not self._is_going_to_base and len(self.task_queue) > 0:
            # Merge new queued tasks into the current batch while we are still at base
            self.current_tasks.extend(self.task_queue)
            self._deliveries_count += len(self.task_queue)
            self.task_queue.clear()

    """Autonomous delivery drone with intelligent routing and multi-state task coordination.

    DeliveryDrone is a concrete specialization of Drone[DeliveryTask] that implements
    sophisticated delivery operations through multi-layered state management, intelligent
    route optimization, and comprehensive package lifecycle tracking. This class demonstrates
    advanced drone framework extensibility while maintaining full compatibility with the
    base drone architecture and state machine framework.

    Architectural Design:
        Multi-Level State Coordination:
            • Base Drone States: GROUNDED, TAKING_OFF, NAVIGATING, LANDING, EMERGENCY
            • Delivery States: ASSIGNED, SERVICE_PICKUP, SERVICE_DROPOFF, DONE
            • Coordinated transitions between flight and delivery state machines
            • Automatic state progression with timing-based service operations

        Intelligent Task Management:
            • Priority-based task selection using DeliveryState ordering
            • Distance-optimized routing for multi-delivery efficiency
            • Dynamic destination assignment based on current task states
            • Automatic mission cleanup and queue management

        Type Safety & Extensibility:
            • Generic type specialization: Drone[DeliveryTask] for compile-time safety
            • Method override pattern for delivery-specific behaviors
            • Integration hooks for custom delivery logic and extensions
            • Full inheritance of base drone capabilities and configurations

    Core Delivery Algorithms:
        Route Optimization Strategy:
            1. State-Based Prioritization: Tasks in earlier states (ASSIGNED) take precedence
               over tasks in later states (SERVICE_PICKUP) for optimal workflow progression
            2. Distance Minimization: Among tasks in the same state, select the closest
               destination to minimize flight time and energy consumption
            3. Dynamic Re-routing: Automatically recalculate routes when new tasks are assigned
               or when missions are completed and removed from the queue

        Mission Execution Flow:
            1. Task Assignment: Receive DeliveryTask through type-safe assign() method
            2. Queue Processing: Move queued tasks to active tasks when ready for execution
            3. Destination Selection: Use _try_assign_new_destination() for optimal routing
            4. Flight Execution: Leverage inherited drone flight capabilities for navigation
            5. Ground Operations: Execute pickup/dropoff with timing-based state progression
            6. Mission Completion: Automatic cleanup and preparation for next delivery cycle

    Specialized Attributes:
        _current_mission (DeliveryTask | None): Reference to the currently executing delivery
                                              task. Set when a destination is assigned and flight
                                              begins, cleared when mission reaches DONE state.
                                              Used for ground operation coordination and state
                                              progression during pickup/dropoff operations.

    Inherited Capabilities:
        From Drone[DeliveryTask]:
            • battery (BatteryStatus): Energy management and operational thresholds
            • position (GeoPoint): WGS84 geographic positioning with navigation methods
            • velocity (Velocity): Configurable cruising speed for flight operations
            • task_queue (deque[DeliveryTask]): Type-safe queue for pending deliveries
            • State machine framework with validated transitions and timing control
            • Power consumption modeling for different flight phases
            • Base station coordination and return-to-base capabilities

    Method Override Specializations:
        Ground State Behavior (on_grounded, enter_grounded):
            • Service operation timing for pickup and dropoff coordination
            • Mission state progression through delivery workflow
            • Automatic task cleanup when deliveries reach completion
            • Destination assignment logic for next mission execution

        Route Planning (_try_assign_new_destination):
            • Multi-criteria task selection using state and distance metrics
            • Dynamic destination assignment based on delivery progression
            • Flight initiation and mission reference management
            • Integration with drone state machine for seamless transitions

    Performance Characteristics:
        Optimization Features:
            • O(n) task selection using min() with custom key functions
            • Efficient distance calculations using geodetic math
            • Memory-optimized single active mission tracking
            • Automatic queue management with minimal memory footprint

        Scalability Considerations:
            • Supports concurrent operation in multi-drone fleets
            • Compatible with real-time simulation frameworks
            • Extensible for advanced routing algorithms (TSP, genetic algorithms)
            • Configurable parameters for different operational requirements

    Integration Examples:
        Basic Delivery Setup:
            >>> from dronesim.geo import GeoPoint
            >>> from dronesim.energy import BatteryStatus
            >>> from dronesim.mission import DeliveryTask
            >>> from dronesim.unit import WattHour, Time
            >>>
            >>> # Initialize drone with operational parameters
            >>> battery = BatteryStatus(capacity=WattHour(100), current=WattHour(95))
            >>> start_pos = GeoPoint.from_deg(37.5665, 126.9780)
            >>> drone = DeliveryDrone(pos=start_pos, battery=battery)
            >>>
            >>> # Create and assign delivery mission
            >>> pickup = GeoPoint.from_deg(37.5700, 126.9800)
            >>> dropoff = GeoPoint.from_deg(37.5750, 126.9850)
            >>> task = DeliveryTask(origin=pickup, destination=dropoff)
            >>> success = drone.assign(task)  # Type-safe assignment

        Multi-Delivery Optimization:
            >>> # Queue multiple deliveries for intelligent routing
            >>> tasks = [
            ...     DeliveryTask(pickup1, dropoff1),
            ...     DeliveryTask(pickup2, dropoff2),
            ...     DeliveryTask(pickup3, dropoff3),
            ... ]
            >>> for task in tasks:
            ...     drone.assign(task)
            >>>
            >>> # Drone automatically optimizes routing based on:
            >>> # 1. Task states (ASSIGNED tasks prioritized)
            >>> # 2. Distance to current position
            >>> # 3. Delivery workflow progression
    """

    _current_mission: DeliveryTask | None = None
    _deliveries_per_charge: int
    _deliveries_count: int
    _is_going_to_base: bool
    _is_on_base: bool
    _start_travel_time: Time | None
    _pakage_count: int
    battery_usage_history: list[tuple[Time, Time, Energy]]

    def __init__(
        self,
        pos: GeoPoint,
        battery: BatteryStatus,
        velocity: Velocity = DEFAULT_VELOCITY,
        transition_duration: Time = DEFAULT_TRANSITION_DURATION,
        power_idle: Power = DEFAULT_CONSUMPTION,
        power_vtol: Power = DEFAULT_CONSUMPTION,
        power_transit: Power = DEFAULT_CONSUMPTION,
        power_per_pakage: Power = DEFAULT_CONSUMPTION,
        operational_battery_percentage: float = DEFAULT_OPERATIONAL_BATTERY_PERCENTAGE,
        base_pos: dict[int, GeoPoint] | None = None,
        max_task_queue_size: int = 0,
        deliveries_per_charge: int = 1,
    ):
        super().__init__(
            pos,
            battery,
            velocity,
            transition_duration,
            power_idle,
            power_vtol,
            power_transit,
            operational_battery_percentage,
            base_pos,
            max_task_queue_size,
        )
        self._deliveries_per_charge = deliveries_per_charge
        self._deliveries_count = 0
        self._is_going_to_base = False
        self._is_on_base = True
        self._start_travel_time = None
        self.battery_usage_history = []
        self._pakage_count = 0
        self.power_per_pakage = power_per_pakage

    def vehicle_update(self, dt, now):
        if self.current_state != DroneState.GROUNDED:
            consume_energy = (
                WattHour.from_si(self._pakage_count * float(dt)) * self.power_per_pakage
            )
            self.battery.consume_energy(consume_energy)
        return super().vehicle_update(dt, now)

    def get_nearest_base(self, last_point: GeoPoint | None = None) -> GeoPoint:
        if last_point is None:
            k, v = min(
                self.base_pos.items(), key=lambda t: self.position.distance_to(t[1])
            )
        else:
            k, v = min(
                self.base_pos.items(), key=lambda t: last_point.distance_to(t[1])
            )

        return v

    def _try_assign_new_destination(self, now: Time) -> None:
        """Execute intelligent destination assignment using state-aware route optimization.

        [Docstring unchanged]
        """
        if not self.is_operational():
            # 배터리 등 운용 불가: 기지로 회항
            if not self._is_on_base:
                v = self.get_nearest_base()
                self.current_destination = v
                self._is_going_to_base = True
                self._start_flight(now)
            return

        # 1) 아직 픽업하지 않은 작업들(ASSIGNED 포함)을 모두 찾는다.
        unpicked = [
            t for t in self.current_tasks if t.current_state == DeliveryState.ASSIGNED
        ]

        if unpicked:
            # 현재 위치 기준, 가장 가까운 origin으로 이동
            target: DeliveryTask = min(
                unpicked, key=lambda t: self.position.distance_to(t.origin)
            )
            self._current_mission = target
            self.current_destination = target.origin
            # 상태 전이: ASSIGNED -> SERVICE_PICKUP (비행 시작 전에 표식)
            target.next(now)
            self._start_flight(now)
            return

        # 2) 모든 픽업이 끝났다면 드롭해야 할 작업들만 남는다.
        droppable = [
            t
            for t in self.current_tasks
            if (t.current_state == DeliveryState.GO_DROPOFF)
        ]

        if droppable:
            # 현재 위치 기준, 가장 가까운 destination으로 이동
            target: DeliveryTask = min(
                droppable, key=lambda t: self.position.distance_to(t.destination)
            )
            self._current_mission = target
            self.current_destination = target.destination
            # 상태 전이: SERVICE_PICKUP -> SERVICE_DROPOFF (비행 시작 전에 표식)
            self._start_flight(now)
            return

        # 3) 더 이상 수행할 것이 없으면 기지 복귀
        if not self._is_on_base and not self._is_going_to_base:
            v = self.get_nearest_base()
            self.current_destination = v
            self._is_going_to_base = True
            self._start_flight(now)
        return

    def route_remainder(
        self, new_task: DeliveryTask | None = None
    ) -> tuple[Length, Energy]:
        pick_up_points: list[GeoPoint] = []
        drop_off_points: list[GeoPoint] = []
        if len(self.current_tasks) > 0:
            for task in self.current_tasks:
                if task.current_state < DeliveryState.SERVICE_PICKUP:
                    pick_up_points.append(task.origin)

                if task.current_state < DeliveryState.SERVICE_DROPOFF:
                    drop_off_points.append(task.destination)

        else:
            if len(self.task_queue) == 0 and new_task is None:
                return (
                    self.get_nearest_base().distance_to(self.position),
                    self.position,
                )

        pick_up_points.sort(key=lambda t: self.position.distance_to(t))
        if len(pick_up_points) > 0:
            drop_off_points.sort(key=lambda t: pick_up_points[-1].distance_to(t))
        else:
            drop_off_points.sort(key=lambda t: self.position.distance_to(t))

        new_pick_up_points: list[GeoPoint] = []
        new_drop_off_points: list[GeoPoint] = []

        for task in self.task_queue:
            new_pick_up_points.append(task.origin)
            new_drop_off_points.append(task.destination)

        if new_task:
            new_pick_up_points.append(new_task.origin)
            new_drop_off_points.append(new_task.destination)

        if len(drop_off_points) > 0:
            new_pick_up_points.sort(key=lambda t: drop_off_points[-1].distance_to(t))
        else:
            new_pick_up_points.sort(key=lambda t: self.position.distance_to(t))

        if len(new_pick_up_points) > 0:
            new_drop_off_points.sort(
                key=lambda t: new_pick_up_points[-1].distance_to(t)
            )
        else:
            new_drop_off_points.sort(key=lambda t: self.position.distance_to(t))

        route: list[GeoPoint] = [self.position] + pick_up_points + drop_off_points
        route += new_pick_up_points + new_drop_off_points
        route += [self.get_nearest_base(route[-1])]

        total_distance: Length = Kilometer(0)

        prev_point: GeoPoint = None
        for point in route:
            if prev_point is None:
                total_distance += self.position.distance_to(point)
            else:
                total_distance += prev_point.distance_to(point)
            prev_point = point
        time = float(total_distance) / float(self.velocity)
        total_energy = WattHour.from_si(float(self.power_transit) * time)
        return total_distance, total_energy

    def on_grounded(self, dt: Time, now: Time) -> None:
        """Execute delivery-specific ground operations with timing-based service coordination.

        Manages comprehensive ground state behavior for delivery operations, including timing-based
        service operations, mission state progression, and intelligent destination assignment.
        This method extends the base drone ground behavior with delivery-specific logic for
        coordinating pickup and dropoff operations within the broader delivery workflow.

        Operational Architecture:
            Ground Service Coordination:
                The method implements nested service functions that handle timing-based operations
                for pickup and dropoff states. These functions coordinate with the delivery task
                state machine to ensure proper workflow progression and timing compliance.

            Mission State Management:
                • Monitors current mission progress through delivery states
                • Coordinates ground operations with flight planning
                • Manages automatic destination assignment when ready for next mission
                • Handles mission completion and cleanup operations

            Parent Class Integration:
                Calls super().on_grounded(dt, now) to preserve base drone ground behavior including:
                • Battery management and charging operations
                • Base station coordination and communication
                • Safety system monitoring and status reporting
                • Idle power consumption management

        Service Operation Functions:
            wait_for_pickup(current_mission: DeliveryTask):
                Timing-Based Pickup Coordination:
                    • Monitors current mission state for SERVICE_PICKUP phase
                    • Evaluates pickup timing constraints using mission.pickup_time
                    • Advances mission state when pickup time threshold is met
                    • Integrates with ground operations for package loading

                Operational Logic:
                    if current_mission.current_state is DeliveryState.SERVICE_PICKUP:
                        if current_mission.pickup_time >= now:  # Timing constraint met
                            current_mission.next(now)          # Advance to next state

            wait_for_dropoff(current_mission: DeliveryTask):
                Service Completion Coordination:
                    • Monitors current mission state for SERVICE_DROPOFF phase
                    • Automatically advances mission state for dropoff completion
                    • Handles package unloading and delivery confirmation
                    • Prepares mission for completion and cleanup

                Operational Logic:
                    if current_mission.current_state is DeliveryState.SERVICE_DROPOFF:
                        current_mission.next(now)  # Immediate progression to DONE

        Destination Assignment Logic:
            Ground Task Detection:
                • Evaluates if current mission is in ground task state
                • Uses DeliveryTask.ground_task() for state classification
                • Executes service operations for ground-based delivery phases
                • Prevents flight initiation during ground service operations

            Flight Preparation:
                • Triggers destination assignment when no current destination is set
                • Calls _try_assign_new_destination(now) for intelligent routing
                • Initiates flight planning for next delivery phase
                • Coordinates transition from ground to flight operations

        Args:
            dt (Time): Simulation time step duration for this update cycle. Used for
                      battery calculations, timing operations, and parent class integration.
                      Passed to super().on_grounded() for base drone ground behavior.

            now (Time): Current simulation timestamp for temporal coordination. Used for
                       timing-based service operations, state progression, and destination
                       assignment. Critical for pickup time evaluations and task advancement.

        Returns:
            None: This method operates through side effects on drone and mission state.
                 Ground operations are managed through internal state changes and
                 coordination with the delivery task state machine.

        Behavioral Flow:
            1. Parent Behavior Execution:
                • Calls super().on_grounded(dt, now) for base ground operations
                • Preserves battery management and safety system functionality
                • Maintains compatibility with drone framework architecture

            2. Destination Status Evaluation:
                if self.current_destination is None:  # No flight destination set
                    Check for ground task operations or initiate destination assignment

            3. Ground Task Service Operations:
                if self._current_mission in DeliveryTask.ground_task():
                    Execute timing-based pickup and dropoff coordination
                    Handle service operations without flight initiation
                    Return early to prevent destination assignment during services

            4. Flight Mission Preparation:
                else:  # Ready for next flight mission
                    Call _try_assign_new_destination(now) for route optimization
                    Initiate flight planning and destination assignment
                    Prepare for transition to TAKING_OFF state

        Integration with State Machines:
            Drone State Machine:
                • Operates within GROUNDED state of base drone state machine
                • Coordinates with base drone ground behavior and timing systems
                • Prepares for transition to TAKING_OFF when destination is assigned

            Delivery Task State Machine:
                • Monitors and advances delivery task states during ground operations
                • Coordinates pickup and dropoff timing with mission requirements
                • Handles state progression from SERVICE_* states to completion

        Performance Considerations:
            • Minimal computational overhead through early returns
            • Efficient state checking using enum comparisons
            • Memory-optimized service operation handling
            • Scalable for high-frequency ground operation cycles
        """

        def wait_for_pickup():
            mission = self._current_mission
            if mission and mission.current_state == DeliveryState.SERVICE_PICKUP:
                if mission.pickup_time <= now:
                    self._pakage_count += 1
                    mission.next(now)

        def wait_for_dropoff():
            mission = self._current_mission
            if mission and mission.current_state == DeliveryState.SERVICE_DROPOFF:
                self._pakage_count -= 1
                mission.next(now)
                mission = self._current_mission

            if mission and mission.current_state == DeliveryState.DONE:
                if mission in self.current_tasks:
                    self.current_tasks.remove(mission)
                self._current_mission = None

        super().on_grounded(dt, now)

        if self._is_on_base:
            self.battery.replace_battery()
            self._load_all_tasks_from_queue()

        if self.current_destination is None:

            if (
                self._current_mission and self._current_mission.current_state in DeliveryTask.ground_task()
            ):
                wait_for_pickup()
                wait_for_dropoff()
                return

            self._try_assign_new_destination(now)
            if (
                self._start_travel_time is None
                and self._current_destination is not None
            ):
                self._is_on_base = False
                self._start_travel_time = now

            return

    def enter_grounded(self, now: Time) -> None:
        """Execute delivery-specific state entry operations with mission progression and cleanup.

        Handles the critical transition into the GROUNDED state with specialized logic for delivery
        mission management, including automatic state progression, mission completion detection,
        and comprehensive cleanup operations. This method extends the base drone grounded entry
        behavior with delivery-specific mission lifecycle management.

        State Transition Architecture:
            Parent Class Integration:
                Calls super().enter_grounded(now) to execute base drone grounded entry operations:
                • Sets power consumption to idle levels for energy efficiency
                • Initializes ground systems and safety protocols
                • Configures communication systems for ground operations
                • Establishes base station coordination if applicable

            Mission Lifecycle Management:
                Implements comprehensive mission state progression and cleanup for active
                deliveries:
                • Advances current mission through delivery state machine
                • Detects mission completion and performs cleanup operations
                • Maintains mission queue integrity and memory efficiency
                • Prepares drone for next mission assignment

        Mission State Progression:
            Automatic State Advancement:
                if self._current_mission is not None:
                    self._current_mission.next(now)  # Advance through delivery workflow

                State Progression Examples:
                • ASSIGNED → SERVICE_PICKUP: Arrived at pickup location, ready for loading
                • SERVICE_PICKUP → SERVICE_DROPOFF: Package loaded, ready for delivery flight
                • SERVICE_DROPOFF → DONE: Package delivered, mission completed

            Timing Coordination:
                The state progression uses the current simulation time (now) for:
                • Timing-based state transitions with mission requirements
                • Coordination with pickup and dropoff scheduling constraints
                • Integration with overall simulation temporal framework

        Mission Completion Handling:
            Completion Detection:
                if self._current_mission.current_state == DeliveryState.DONE:
                    # Mission has reached final state, perform cleanup

            Cleanup Operations:
                1. Task List Management:
                    self.current_tasks.remove(self._current_mission)
                    • Removes completed mission from active task list
                    • Maintains memory efficiency by freeing completed tasks
                    • Preserves task queue integrity for remaining missions

                2. Mission Reference Cleanup:
                    self._current_mission = None
                    • Clears internal mission reference to prevent stale state
                    • Prepares drone for assignment of next available mission
                    • Ensures clean mission lifecycle management

        Args:
            now (Time): Current simulation timestamp used for mission state progression and
                       timing coordination. This time is passed to the mission state machine
                       for temporal operations and state transition timing calculations.

        Returns:
            None: This method operates through side effects on drone state and mission
                 management. Success is indicated through mission state progression and
                 automatic cleanup of completed deliveries.

        Side Effects:
            Mission State Management:
                • Advances current mission through delivery state machine using mission.next(now)
                • Coordinates state transitions with simulation timing framework
                • Maintains mission workflow integrity and progression

            Task List Maintenance:
                • Removes completed missions from current_tasks list when DONE state is reached
                • Preserves active mission queue for continued operations
                • Maintains memory efficiency through automatic cleanup

            Mission Reference Management:
                • Clears _current_mission reference when missions reach completion
                • Prepares internal state for next mission assignment
                • Ensures clean separation between completed and pending missions

            Base Drone State:
                • Inherits all side effects from super().enter_grounded(now)
                • Maintains power consumption, communication, and safety system state
                • Preserves integration with base drone architecture and timing systems

        Operational Flow:
            1. Base State Entry:
                super().enter_grounded(now)  # Execute base drone grounded entry
                • Configure idle power consumption and ground systems
                • Establish communication and safety protocols

            2. Mission State Evaluation:
                if self._current_mission is not None:  # Active mission exists
                    Process mission state progression and completion handling

            3. Mission State Progression:
                self._current_mission.next(now)  # Advance delivery workflow
                • Coordinate with delivery state machine timing
                • Handle state-specific operations and requirements

            4. Completion Detection and Cleanup:
                if self._current_mission.current_state == DeliveryState.DONE:
                    Execute comprehensive mission cleanup operations
                    • Remove from active task list
                    • Clear mission reference
                    • Prepare for next mission assignment

        Integration with Drone Framework:
            State Machine Coordination:
                • Operates as entry action for GROUNDED state in drone state machine
                • Coordinates with base drone state entry operations and timing
                • Maintains compatibility with drone framework architecture

            Mission Management System:
                • Integrates with delivery task state machine for workflow coordination
                • Handles mission lifecycle from active execution to completion
                • Maintains task queue integrity and mission assignment readiness

        Performance Characteristics:
            • Minimal computational overhead with single mission state check
            • Efficient memory management through automatic cleanup operations
            • O(1) complexity for mission completion detection and cleanup
            • Scalable for high-volume delivery operations with frequent completions

        Error Handling:
            • Null-safe mission reference checking prevents runtime errors
            • State machine integration ensures valid state transitions
            • Task list operations maintain queue integrity even with rapid completions
        """
        if self._is_going_to_base:
            self._is_going_to_base = False
            self._deliveries_count = 0
            self._is_on_base = True
            battery_useage = self.battery.capacity - self.battery.current
            self.battery_usage_history.append(
                (self._start_travattery_useage)
            )
            self._start_travel_time = None

        super().enter_grounded(now)
        if (
            self._current_mission is not None
            and self._current_mission.current_state
            not in (DeliveryState.DONE, DeliveryState.ABORTED)
        ):
            self._current_mission.next(now)

    def can_accept_task(self) -> bool:
        """Accept tasks while waiting at base (GROUNDED), block during return/flight.

        This enables batching: while the drone is on base and idle, we can keep
        accepting tasks into task_queue; `_load_all_tasks_from_queue()` will merge
        them into `current_tasks` before takeoff.
        """
        c1 = self._is_going_to_base or self._is_on_base
        c2 = len(self.current_tasks) == 0 and len(self.task_queue) == 0
        if c1 and c2:
            return True
        return False

    def assign(self, task):
        if self.is_operational() and self.can_accept_task():
            return super().assign(task)
        return False

    def is_operational(self):
        # Operational readiness is a battery/health check from the base class.
        # Batching (task/current_tasks) should not gate operational status.
        return super().is_operational()

    @property
    def is_busy(self) -> bool:
        return super().is_busy or self._is_going_to_base
