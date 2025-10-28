"""Autonomous drone vehicle implementation with state machine framework.

This module implements the Drone class, a concrete vehicle type that extends
the abstract Vehicle base class with comprehensive state machine integration.
It provides drone-specific functionality including battery-powered flight operations,
state-based behavior management, and customizable operational logic through
overrideable methods.

The Drone class represents quadcopter-style autonomous vehicles with a finite
state machine controlling flight phases (GROUNDED, TAKING_OFF, NAVIGATING, LANDING, EMERGENCY).
Each state has associated behavior methods that can be overridden by subclasses
for specialized drone implementations.

Key Features:
    • State machine-driven flight operations (DroneState enum)
    • Overrideable state behavior methods (on_*, enter_* patterns)
    • Battery-powered operations with multi-phase power consumption
    • Geographic positioning with WGS84 coordinate system
    • Generic task management with type safety
    • Base station coordination and operational thresholds
    • Customizable update logic for specialized drone types

Extensibility:
    The Drone class is designed for extension through method overriding:
    • vehicle_update(dt): Main update loop - override for custom behavior
    • on_*() methods: State behavior handlers - override for state-specific logic
    • enter_*() methods: State entry actions - override for transition logic

    This pattern allows specialized drone types (delivery, surveillance, etc.)
    to customize behavior while maintaining the core state machine framework.

State Machine Integration:
    Uses the app.state.StateMachine framework for validated state transitions
    with configurable actions and effects. State transitions are enforced
    through the transition validation system.

Example Usage:
    >>> from app.geo import GeoPoint
    >>> from app.energy.battery import BatteryStatus
    >>> from app.energy.unit import WattHour
    >>> from app.unit import KilometersPerHour, Watt
    >>>
    >>> # Create drone with custom power profile
    >>> drone = Drone(
    ...     pos=GeoPoint.from_deg(37.5665, 126.9780),
    ...     battery=BatteryStatus(WattHour(1000), WattHour(800)),
    ...     velocity=KilometersPerHour(60.0),
    ...     power_idle=Watt(50.0),
    ...     power_vtol=Watt(500.0),
    ...     power_transit=Watt(200.0),
    ... )
    >>>
    >>> # Override for specialized behavior
    >>> class DeliveryDrone(Drone[DeliveryTask]):
    ...     def on_navigating(self, dt: Time):
    ...         super().on_navigating(dt)  # Call base behavior
    ...         self.update_delivery_status()  # Add custom logic
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto

from app.energy import BatteryStatus
from app.geo import GeoPoint
from app.mission import Task
from app.state import Action, ActionFn
from app.timer import Timer
from app.unit import KilometersPerHour, Meter, Minute, Power, Time, Velocity, Watt

from .vehicle import Vehicle

DEFAULT_VELOCITY = KilometersPerHour(50.0)
DEFAULT_TRANSITION_DURATION = Minute(1.0)
DEFAULT_CONSUMPTION = Watt(0.0)
DEFAULT_OPERATIONAL_BATTERY_PERCENTAGE = 20.0  # Minimum battery percentage to be operational


class DroneState(Enum):
    GROUNDED = auto()
    TAKING_OFF = auto()
    NAVIGATING = auto()
    LANDING = auto()
    EMERGENCY = auto()


@dataclass(frozen=True)
class PowerConsumption:
    """Represents the power consumption profile of a drone across different operational modes.

    This immutable data structure encapsulates the three primary power consumption
    states of drone operations, providing a comprehensive energy model for accurate
    battery life estimation and mission planning.

    Attributes:
        idle (Power): Power consumption during stationary operations such as hovering,
                     sensor data collection, and standby modes.
        vtol (Power): Power consumption during Vertical Take-Off and Landing operations,
                     typically the highest power draw due to motor load requirements.
        transit (Power): Power consumption during horizontal flight operations at
                        cruising speed, optimized for energy efficiency over distance.
    """

    idle: Power
    vtol: Power
    transit: Power


@dataclass(frozen=True)
class DroneStatus[T: Task]:
    """Immutable snapshot of drone operational state and configuration.

    This generic frozen dataclass provides a comprehensive view of a drone's current
    status including position, energy state, performance parameters, and
    task assignments. Used for monitoring, logging, and decision-making
    within the simulation environment.

    Type Parameters:
        T: Type of tasks handled by this drone, must be a subclass of Task.

    Attributes:
        id (int): Unique identifier for the drone instance.
        battery (BatteryStatus): Current battery charge level and capacity information.
        position (GeoPoint): Current geographic position using WGS84 coordinates.
        power (PowerConsumption): Power consumption profile for different operational modes.
        velocity (Velocity): Current configured cruising speed for horizontal movement.
        transition_duration (Time): Time required for operational state transitions.
        task_current (list[T] | None): Currently executing task assignments of type T,
                                      None if no active tasks.
        task_queue (Deque[T]): Double-ended queue of tasks of type T waiting for execution,
                              providing efficient FIFO/LIFO operations.
    """

    id: int
    battery: BatteryStatus
    position: GeoPoint
    power: PowerConsumption
    velocity: Velocity
    transition_duration: Time

    task_current: list[T] | None
    task_queue: deque[T]


class Drone[T: Task](Vehicle):
    """Generic autonomous drone with state machine-driven behavior and extensible methods.

    A state machine-controlled quadcopter that extends Vehicle with comprehensive
    flight operations, battery management, and overrideable behavior methods.
    Designed for extension through method overriding to create specialized drone types.

    State Machine Architecture:
        The drone operates through five primary states (DroneState enum):
        - GROUNDED: On ground, idle power consumption
        - TAKING_OFF: Vertical ascent, high power consumption
        - NAVIGATING: Horizontal flight, moderate power consumption
        - LANDING: Vertical descent, high power consumption
        - EMERGENCY: Emergency procedures, variable power consumption

    Extensibility Through Method Overriding:
        This class provides hook methods that subclasses can override for customization:

        Core Update Method:
        • vehicle_update(dt): Main update loop called each simulation step
          Override to implement specialized drone behavior and logic

        State Behavior Methods (called continuously while in state):
        • on_grounded(dt): Override for ground operations (charging, maintenance)
        • on_taking_off(dt): Override for takeoff procedures and checks
        • on_navigating(dt): Override for flight behavior and task execution
        • on_landing(dt): Override for landing procedures and positioning
        • on_emergency(dt): Override for emergency response protocols

        State Entry Methods (called once when entering state):
        • enter_grounded(dt): Override for ground state initialization
        • enter_taking_off(dt): Override for takeoff state setup
        • enter_navigating(dt): Override for navigation state initialization
        • enter_landing(dt): Override for landing state setup
        • enter_emergency(dt): Override for emergency state activation

    Type Parameters:
        T: Task type this drone handles, must extend Task base class.
           Enables type-safe task assignment and specialized behaviors.

    Key Attributes:
        battery: Current battery status and capacity information
        operational_battery_percentage: Minimum battery threshold for operations
        velocity: Cruising speed for horizontal movement
        transition_duration: Time for state transitions (takeoff/landing)
        power_idle/vtol/transit: Power consumption for different operational phases
        base_pos: Dictionary of base station positions

    Private Attributes:
        _task_current: Currently executing tasks of type T
        _task_queue: Queue of pending tasks for execution
        _current_destination: Target position for movement
        _battery_usage: Current power consumption rate
        _vlot_timer: Timer for vertical operations (takeoff/landing)

    Usage Pattern:
        class SpecializedDrone(Drone[MyTaskType]):
            def vehicle_update(self, dt: Time):
                super().vehicle_update(dt)  # Call base behavior
                self.my_custom_logic()      # Add specialized logic

            def on_navigating(self, dt: Time):
                super().on_navigating(dt)   # Base navigation behavior
                self.execute_specialized_navigation()  # Custom navigation
    """

    battery: BatteryStatus
    operational_battery_percentage: float
    base_pos: dict[GeoPoint]

    velocity: Velocity
    transition_duration: Time

    power_idle: Power
    power_vtol: Power
    power_transit: Power

    _tasks_current: list[T] | None
    _task_queue: deque[T]

    _current_destination: GeoPoint | None
    _on_actions: dict[DroneState, ActionFn]
    _battery_usage: Power
    _vlot_timer: Timer | None

    def __init__(
        self,
        pos: GeoPoint,
        battery: BatteryStatus,
        velocity: Velocity = DEFAULT_VELOCITY,
        transition_duration: Time = DEFAULT_TRANSITION_DURATION,
        power_idle: Power = DEFAULT_CONSUMPTION,
        power_vtol: Power = DEFAULT_CONSUMPTION,
        power_transit: Power = DEFAULT_CONSUMPTION,
        operational_battery_percentage: float = DEFAULT_OPERATIONAL_BATTERY_PERCENTAGE,
        base_pos: dict[GeoPoint] | None = None,
    ):
        """Initialize a new Drone instance with operational parameters.

        Creates a new autonomous drone with specified performance characteristics,
        power consumption model, and base station configuration. The drone inherits
        positioning capabilities from the Vehicle base class and extends it with
        battery management and multi-phase power consumption modeling.

        Args:
            pos (GeoPoint): Initial geographic position using WGS84 coordinates.
            battery (BatteryStatus): Battery status including current charge and capacity.
            velocity (Velocity): Cruising speed for horizontal flight operations.
                Defaults to 50 km/h if not specified.
            transition_duration (Time): Time required for state transitions such as
                takeoff, landing, and mode changes. Defaults to 1 minute.
            power_idle (Power): Power consumption during idle and hovering operations.
                Defaults to 0W if not specified.
            power_vtol (Power): Power consumption during vertical takeoff and landing
                operations. Defaults to 0W if not specified.
            power_transit (Power): Power consumption during horizontal flight operations.
                Defaults to 0W if not specified.
            operational_battery_percentage (float): Minimum battery percentage threshold
                for operational status. Drone is considered non-operational below this
                threshold. Must be between 0.0 and 100.0. Defaults to 20%.
            base_pos (dict[GeoPoint] | None): Dictionary of base station positions
                keyed by unique identifiers. If None, creates a single base station
                at the initial position.

        Raises:
            ValueError: If operational_battery_percentage is not between 0.0 and 100.0.

        Note:
            The power consumption parameters enable realistic energy modeling for
            different flight phases. Setting appropriate values is crucial for
            accurate mission planning and battery life estimation.
        """
        super().__init__(pos)
        self.battery = battery

        with operational_battery_percentage as percent:
            if percent < 0.0 or percent > 100.0:
                msg = f"Invalid operational battery percentage: {percent}"
                raise ValueError(msg)

        self.operational_battery_percentage = operational_battery_percentage

        self.velocity = velocity
        self.transition_duration = transition_duration
        self.power_idle = power_idle
        self.power_vtol = power_vtol
        self.power_transit = power_transit

        if not base_pos:
            copy_pos = pos.copy()
            base_pos = {copy_pos.id: copy_pos}
        self.base_pos = base_pos
        self._task_queue = deque()
        self._tasks_current = None
        self._battery_usage = self.power_idle

        self._current_destination = None
        self.init_state_machine(
            DroneState.GROUNDED,
            {
                DroneState.GROUNDED: [Action(DroneState.TAKING_OFF, self.enter_taking_off)],
                DroneState.TAKING_OFF: [
                    Action(DroneState.NAVIGATING, self.enter_navigating),
                    Action(DroneState.EMERGENCY, self.enter_emergency),
                ],
                DroneState.NAVIGATING: [
                    Action(DroneState.LANDING, self.enter_landing),
                    Action(DroneState.EMERGENCY, self.enter_emergency),
                ],
                DroneState.LANDING: [
                    Action(DroneState.GROUNDED, self.enter_grounded),
                    Action(DroneState.EMERGENCY, self.enter_emergency),
                ],
                DroneState.EMERGENCY: [Action(DroneState.GROUNDED, self.enter_grounded)],
            },
        )

        self._on_actions = {
            DroneState.GROUNDED: self.on_grounded,
            DroneState.TAKING_OFF: self.on_taking_off,
            DroneState.NAVIGATING: self.on_navigating,
            DroneState.LANDING: self.on_landing,
            DroneState.EMERGENCY: self.on_emergency,
        }

    @property
    def current_destination(self) -> GeoPoint | None:
        """Get the current destination point for the drone.

        Returns:
            GeoPoint | None: The current destination point or None if not set.
        """
        return self._current_destination

    @current_destination.setter
    def current_destination(self, destination: GeoPoint | None):
        """Set the current destination point for the drone.

        Args:
            destination (GeoPoint | None): The new destination point or None to clear.
        """
        if self._current_destination is not None:
            msg = "Current destination is already set."
            raise ValueError(msg)

        self._current_destination = destination

    @property
    def current_tasks(self) -> T | None:
        """Get the current task assigned to the drone.

        Returns:
            T | None: The current task of type T or None if no task is assigned.
        """
        return self._tasks_current

    @current_tasks.setter
    def current_tasks(self, task: T | None):
        """Set the current task assigned to the drone.

        Args:
            task (T | None): The new task of type T or None to clear the current task.
        """
        self._tasks_current = task

    @property
    def task_queue(self) -> deque[T]:
        """Get the task queue for the drone.

        Returns:
            Deque[T]: The double-ended queue of tasks of type T.
        """
        return self._task_queue

    def go_to_destination(self, dt: Time) -> bool:
        """Move the drone towards its current destination and check for arrival.

        Calculates the heading to the current destination and moves the drone
        towards it based on the configured velocity and given time delta.
        Uses geometric path analysis to detect when the drone has reached or
        passed through the target destination during this movement step.

        The method implements intelligent destination detection by checking if
        the movement path from the current position to the next position passes
        through the target location. Upon reaching the destination, the drone's
        position is precisely set to the target and the current destination is
        automatically cleared.

        Args:
            dt (Time): Time step duration for movement calculation. The actual
                      movement distance is calculated as velocity * dt.

        Returns:
            bool: True if the drone successfully reached the destination during
                  this step, False if still traveling or no destination is set.

        Note:
            This method requires a destination to be set via the current_destination
            property. If no destination is set, no movement occurs and False is returned.
            When the destination is reached, current_destination is automatically
            set to None, requiring a new destination to be assigned for further movement.
        """
        if self._current_destination is None:
            return False

        azimuth = self.position.heading_to(self._current_destination)
        next_position = self.position.forward(azimuth, Meter(float(self.velocity) * float(dt)))

        # Check if the path to the next position passes through the target
        if next_position.passed_through_target(
            start=self.position,
            target=self._current_destination,
        ):
            self.position = self._current_destination
            self._current_destination = None
            return True

        # Continue moving towards destination
        self.position = next_position
        return False

    def get_status(self) -> DroneStatus[T]:
        """Get current drone status information.

        Returns:
            DroneStatus[T]: Generic DroneStatus object containing drone status information
                           including position, battery level, and task queue of type T.
        """
        return DroneStatus(
            id=self.id,
            battery=self.battery,
            position=self.position,
            power=PowerConsumption(
                idle=self.power_idle,
                vtol=self.power_vtol,
                transit=self.power_transit,
            ),
            velocity=self.velocity,
            transition_duration=self.transition_duration,
            task_current=self._tasks_current,
            task_queue=self._task_queue,
        )

    def is_operational(self) -> bool:
        """Check if the drone is operational (has sufficient battery).

        Returns:
            bool: True if the drone has sufficient battery to operate, False otherwise.
        """
        # Consider drone operational if battery is above operational threshold
        return self.battery.percentage > self.operational_battery_percentage

    def assign(self, task: T) -> bool:
        """Assign a task to this drone.

        Evaluates whether this drone can accept the given task based on
        battery levels, current position, and operational constraints.
        The task type must match the drone's generic type parameter T.

        Args:
            task (T): The task of type T to be assigned to this drone.

        Returns:
            bool: True if the task was successfully assigned, False otherwise.
        """
        # Simple implementation - check if drone is operational
        if not self.is_operational():
            return False
        self._task_queue.append(task)
        return True

    def vehicle_update(self, dt):
        """Execute drone-specific update logic for one simulation step.

        This method is called each simulation timestep and handles core drone
        state machine execution. The base implementation dispatches to appropriate
        state handler methods based on the current state.

        Override this method in subclasses to implement specialized drone behavior:
        - Add custom logic before/after state handling
        - Implement task-specific update operations
        - Integrate with external systems or sensors
        - Perform specialized monitoring or logging

        Args:
            dt (Time): Time step duration in simulation time units.

        Example Override:
            def vehicle_update(self, dt):
                super().vehicle_update(dt)  # Call base state machine logic
                self.update_sensor_data()   # Custom functionality
                self.check_mission_status() # Specialized behavior
        """
        current_state = self.current_state
        if current_state in self._on_actions:
            self._on_actions[current_state](dt)

    def on_grounded(self, dt: Time) -> None:
        """Handle drone behavior while in the GROUNDED state.

        Called continuously while the drone remains on the ground. The base
        implementation provides default idle behavior.

        Override this method to implement specialized ground operations:
        - Battery charging logic
        - Task loading and preparation
        - Pre-flight system checks
        - Ground-based maintenance routines
        - Communication with ground control

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def on_grounded(self, dt: Time):
                super().on_grounded(dt)     # Call base behavior
                self.charge_battery(dt)     # Custom charging logic
                self.perform_diagnostics()  # Specialized checks
        """
        pass

    def enter_grounded(self, dt: Time) -> None:
        """Handle state entry actions when transitioning to GROUNDED state.

        Called once when the drone enters the GROUNDED state. The base
        implementation sets power consumption to idle levels.

        Override this method to implement specialized ground state initialization:
        - State transition logging
        - Ground systems activation
        - Task completion notifications
        - Battery status reporting
        - Ground control registration

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def enter_grounded(self, dt: Time):
                super().enter_grounded(dt)      # Set idle power consumption
                self.log_landing_complete()     # Custom logging
                self.register_with_ground()     # Ground control integration
                self.report_mission_status()    # Mission completion reporting
        """
        self._battery_usage = self.power_idle

    def on_taking_off(self, dt: Time) -> None:
        """Handle drone behavior while in the TAKING_OFF state.

        Called continuously during vertical takeoff operations. The base
        implementation manages takeoff timing and transitions to NAVIGATING
        when the takeoff timer completes.

        Override this method to implement specialized takeoff behavior:
        - Custom ascent rate control
        - Takeoff safety checks
        - Altitude monitoring during ascent
        - Environmental condition responses
        - Payload stabilization during takeoff

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def on_taking_off(self, dt: Time):
                super().on_taking_off(dt)   # Handle base timing logic
                self.monitor_altitude()     # Custom altitude tracking
                self.stabilize_payload()    # Specialized payload handling
        """
        if self._vlot_timer and self._vlot_timer.done:
            self.transition_to(DroneState.NAVIGATING)
            self._vlot_timer = None

    def enter_taking_off(self, dt: Time) -> None:
        """Handle state entry actions when transitioning to TAKING_OFF state.

        Called once when the drone enters the TAKING_OFF state. The base
        implementation sets VTOL power consumption and starts the takeoff timer.

        Override this method to implement specialized takeoff initialization:
        - Takeoff procedure logging
        - System checks and validations
        - Flight plan activation
        - Navigation system initialization
        - Communication protocol setup

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def enter_taking_off(self, dt: Time):
                super().enter_taking_off(dt)    # Set VTOL power and timer
                self.log_takeoff_initiated()    # Custom logging
                self.validate_flight_systems()  # Pre-flight checks
                self.activate_navigation()      # Navigation system startup
        """
        self._battery_usage = self.power_vtol + self.power_idle
        self._vlot_timer = self.create_timer(self.transition_duration)

    def on_navigating(self, dt: Time) -> None:
        """Handle drone behavior while in the NAVIGATING state.

        Called continuously during horizontal flight operations. The base
        implementation handles movement toward the current destination and
        transitions to LANDING when the destination is reached.

        Override this method to implement specialized navigation behavior:
        - Custom waypoint following algorithms
        - Task execution during flight
        - Obstacle avoidance systems
        - Dynamic route adjustment
        - Payload operations during transit
        - Formation flight coordination

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def on_navigating(self, dt: Time):
                super().on_navigating(dt)   # Handle base movement
                self.execute_tasks()        # Process active tasks
                self.avoid_obstacles()      # Custom obstacle avoidance
                self.update_formation()     # Coordinate with other drones
        """
        if self.go_to_destination(dt):
            self.transition_to(DroneState.LANDING)

    def enter_navigating(self, dt: Time) -> None:
        """Handle state entry actions when transitioning to NAVIGATING state.

        Called once when the drone enters the NAVIGATING state. The base
        implementation sets transit power consumption for horizontal flight.

        Override this method to implement specialized navigation initialization:
        - Navigation mode activation logging
        - Flight path calculation
        - Task execution preparation
        - Waypoint system initialization
        - Communication frequency setup

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def enter_navigating(self, dt: Time):
                super().enter_navigating(dt)    # Set transit power consumption
                self.log_navigation_started()   # Custom logging
                self.calculate_optimal_path()   # Route optimization
                self.initialize_tasks()         # Task system preparation
        """
        self._battery_usage = self.power_transit + self.power_idle

    def on_landing(self, dt: Time) -> None:
        """Handle drone behavior while in the LANDING state.

        Called continuously during vertical landing operations. The base
        implementation manages landing timing and transitions to GROUNDED
        when the landing timer completes.

        Override this method to implement specialized landing behavior:
        - Precision landing control
        - Landing zone verification
        - Payload delivery operations
        - Ground proximity sensors
        - Landing safety protocols
        - Post-flight procedures initialization

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def on_landing(self, dt: Time):
                super().on_landing(dt)      # Handle base timing logic
                self.verify_landing_zone()  # Custom safety checks
                self.deploy_payload()       # Specialized payload handling
                self.prepare_shutdown()     # Post-landing procedures
        """
        if self._vlot_timer and self._vlot_timer.done:
            self.transition_to(DroneState.GROUNDED)
            self._vlot_timer = None

    def enter_landing(self, dt: Time) -> None:
        """Handle state entry actions when transitioning to LANDING state.

        Called once when the drone enters the LANDING state. The base
        implementation starts the landing timer for vertical descent operations.

        Override this method to implement specialized landing initialization:
        - Landing procedure activation
        - Landing zone analysis
        - Descent rate configuration
        - Safety system checks
        - Ground communication establishment

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def enter_landing(self, dt: Time):
                super().enter_landing(dt)       # Start landing timer
                self.log_landing_initiated()    # Custom logging
                self.analyze_landing_zone()     # Landing site verification
                self.configure_descent_rate()   # Descent parameter setup
        """
        self._vlot_timer = self.create_timer(self.transition_duration)

    def on_emergency(self, dt: Time) -> None:
        """Handle drone behavior while in the EMERGENCY state.

        Called continuously during emergency situations. The base implementation
        provides minimal emergency behavior framework.

        Override this method to implement specialized emergency protocols:
        - Emergency landing procedures
        - Distress signal transmission
        - System diagnostics and recovery
        - Payload jettison procedures
        - Emergency communication protocols
        - Failsafe activation sequences

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def on_emergency(self, dt: Time):
                super().on_emergency(dt)        # Base emergency framework
                self.transmit_distress()        # Custom emergency communication
                self.execute_emergency_landing() # Specialized landing protocol
                self.activate_beacon()          # Emergency location beacon
        """
        pass

    def enter_emergency(self, dt: Time) -> None:
        """Handle state entry actions when transitioning to EMERGENCY state.

        Called once when the drone enters the EMERGENCY state. The base
        implementation provides minimal emergency state setup framework.

        Override this method to implement specialized emergency initialization:
        - Emergency protocol activation
        - Distress signal transmission
        - System shutdown procedures
        - Emergency landing site selection
        - Critical system diagnostics
        - Emergency communication setup

        Args:
            dt (Time): Time step duration for simulation update.

        Example Override:
            def enter_emergency(self, dt: Time):
                super().enter_emergency(dt)         # Base emergency setup
                self.log_emergency_declared()       # Emergency logging
                self.transmit_mayday()              # Distress signal
                self.activate_emergency_beacon()    # Location beacon
                self.shutdown_non_critical()        # System preservation
        """
        pass
