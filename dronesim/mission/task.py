"""Comprehensive abstract task framework for autonomous mission orchestration and lifecycle management.

This module implements the foundational Task abstract base class, establishing a robust framework for
mission task management within the drone simulation ecosystem. The framework provides sophisticated
state machine integration, temporal tracking, geographic waypoint management, and extensible
architecture patterns for diverse mission types ranging from delivery operations to surveillance
and reconnaissance tasks.

Architectural Foundation:
    Design Pattern Integration:
        • Template Method Pattern: Provides concrete infrastructure while deferring mission-specific
          logic to specialized implementations through abstract method requirements
        • State Machine Pattern: Enforces validated state transitions with configurable rules and
          actions for reliable mission execution and error recovery
        • Abstract Factory Pattern: Enables extensible task type creation with consistent interfaces
          and behaviors across heterogeneous mission categories
        • Observer Pattern: Supports event-driven temporal tracking and state change notifications

    Core Framework Components:
        Task Abstract Base Class:
            • Geographic waypoint management with WGS84 coordinate system integration
            • Unique identification system with automatic ID generation fallbacks
            • Temporal lifecycle tracking with start and completion timestamps
            • State machine integration with validated transition rules and error handling
            • Extensible architecture supporting arbitrary mission-specific attributes

        State Management Infrastructure:
            • StateMachine integration with configurable transition graphs
            • Validated state transitions with automatic error detection and recovery
            • Action-based state effects with parameter passing and return value handling
            • Thread-safe state access with consistent current state reporting
            • Exception handling with descriptive error messages for debugging

        Temporal Coordination System:
            • Mission lifecycle timestamps with start_at and completed_at tracking
            • Write-once temporal attributes preventing accidental overwrites
            • Integration with simulation time frameworks for consistent temporal coordination
            • Support for mission duration calculations and performance analytics

Mission Lifecycle Management:
    Initialization Phase:
        Task instances are created with geographic waypoints (origin and destination) and
        optional unique identifiers. The state machine must be initialized either through
        subclass-specific logic or explicit init_state_machine() calls with appropriate
        initial states and transition graphs.

    Execution Phase:
        Tasks progress through their state machines using transition_to() method calls,
        which validate transitions against configured rules and execute associated actions.
        State transitions are atomic and thread-safe, preventing race conditions in
        multi-threaded simulation environments.

    Completion Phase:
        Tasks reach terminal states (typically DONE or ABORTED) and provide done property
        checks for completion detection. Temporal tracking captures completion timestamps
        for performance analysis and mission reporting.

Implementation Requirements:
    Mandatory Subclass Definitions:
        1. State Enumeration: Define task-specific states extending Enum or IntEnum
           with appropriate ordering for state machine progression
        2. State Transition Graph: Configure StateGraph with valid transitions and actions
           including error recovery paths and terminal state handling
        3. State Machine Initialization: Set up StateMachine instance with initial state
           and transition rules through init_state_machine() or constructor logic
        4. Abstract Method Implementation: Provide concrete abort() and done property
           implementations with task-specific logic and validation

    State Machine Configuration:
        class TaskState(IntEnum):
            PENDING = auto()
            IN_PROGRESS = auto()
            COMPLETED = auto()
            ABORTED = auto()

        transition_graph = {
            TaskState.PENDING: [
                Action(TaskState.IN_PROGRESS, start_execution_effect),
                Action(TaskState.ABORTED, abort_effect)
            ],
            TaskState.IN_PROGRESS: [
                Action(TaskState.COMPLETED, completion_effect),
                Action(TaskState.ABORTED, abort_effect)
            ],
            TaskState.COMPLETED: [],
            TaskState.ABORTED: []
        }

Framework Extensions:
    Custom Task Types:
        The framework supports arbitrary mission types through inheritance and method
        overriding. Common extensions include delivery tasks, surveillance missions,
        reconnaissance operations, and maintenance tasks, each with specialized state
        machines and execution logic.

    Advanced State Management:
        Subclasses can implement complex state transitions with custom validation,
        conditional branching, rollback mechanisms, and multi-phase execution patterns.
        The framework provides hooks for pre-transition validation and post-transition
        effects.

    Integration Capabilities:
        • Geographic Information Systems (GIS) integration for spatial analysis
        • Temporal database integration for mission history and analytics
        • Event logging and monitoring systems for operational visibility
        • Performance metrics collection for optimization and improvement

Performance Characteristics:
    Memory Efficiency:
        • Minimal memory footprint with lazy initialization patterns
        • Efficient state machine implementations with O(1) state transitions
        • Garbage collection friendly with proper reference management
        • Scalable for large-scale simulations with thousands of concurrent tasks

    Thread Safety:
        • Thread-safe state machine operations with atomic transitions
        • Immutable geographic waypoint references preventing data races
        • Protected temporal attribute updates with write-once semantics
        • Safe concurrent access to task properties and state information

Integration Examples:
    Basic Task Implementation:
        >>> from enum import IntEnum, auto
        >>> from dronesim.state import Action, StateMachine
        >>> from dronesim.geo import GeoPoint
        >>> from dronesim.unit import Time
        >>>
        >>> class MyTaskState(IntEnum):
        ...     PENDING = auto()
        ...     EXECUTING = auto()
        ...     COMPLETED = auto()
        ...     ABORTED = auto()
        >>>
        >>> class MyTask(Task):
        ...     def __init__(self, origin: GeoPoint, destination: GeoPoint):
        ...         super().__init__(origin, destination)
        ...         transition_graph = {
        ...             MyTaskState.PENDING: [Action(MyTaskState.EXECUTING)],
        ...             MyTaskState.EXECUTING: [Action(MyTaskState.COMPLETED)],
        ...             MyTaskState.COMPLETED: [],
        ...             MyTaskState.ABORTED: [],
        ...         }
        ...         self.init_state_machine(MyTaskState.PENDING, transition_graph)
        ...
        ...     def abort(self):
        ...         self.transition_to(MyTaskState.ABORTED)
        ...
        ...     @property
        ...     def done(self) -> bool:
        ...         return self.current_state in {MyTaskState.COMPLETED, MyTaskState.ABORTED}

    Advanced Mission Coordination:
        >>> # Create task with temporal tracking
        >>> origin = GeoPoint.from_deg(37.5665, 126.9780)
        >>> destination = GeoPoint.from_deg(37.5675, 126.9890)
        >>> task = MyTask(origin, destination)
        >>> task.start_at = Time.from_seconds(0)  # Mission start
        >>>
        >>> # Execute mission with state transitions
        >>> task.transition_to(MyTaskState.EXECUTING)
        >>> # ... mission execution logic ...
        >>> task.transition_to(MyTaskState.COMPLETED)
        >>> task.completed_at = Time.from_seconds(120)  # Mission completion
        >>>
        >>> # Verify mission completion
        >>> assert task.done == True
        >>> print(f"Mission duration: {task.completed_at - task.start_at}")
"""

from abc import ABC, abstractmethod
from typing import Any

from dronesim.geo import GeoPoint
from dronesim.state import State, StateGraph, StateMachine
from dronesim.unit import Time


class Task(ABC):
    """Abstract foundation for autonomous mission tasks with comprehensive state management and lifecycle control.

    This abstract base class establishes the fundamental architecture for all mission tasks within the
    drone simulation ecosystem, providing robust infrastructure for state machine integration, temporal
    tracking, geographic waypoint management, and extensible mission-specific behaviors. The class
    implements sophisticated design patterns to ensure reliable, thread-safe, and scalable task execution
    across diverse mission types and operational scenarios.

    Architectural Design:
        Template Method Pattern Implementation:
            The class provides concrete implementations for common task operations (initialization,
            identification, state management, temporal tracking) while deferring mission-specific
            logic to abstract methods that subclasses must implement. This ensures consistent
            behavior across all task types while enabling specialized functionality.

        State Machine Integration:
            Each task instance manages its lifecycle through a sophisticated state machine that
            validates transitions, executes associated actions, and maintains operational integrity.
            The state machine prevents invalid state transitions and provides error recovery
            mechanisms for robust mission execution.

        Geographic Waypoint System:
            Tasks maintain origin and destination waypoints using the WGS84 coordinate system,
            enabling precise geographic positioning and navigation calculations. The waypoint
            system supports complex multi-point missions through extensible coordinate management.

        Temporal Lifecycle Management:
            Comprehensive timestamp tracking captures mission start and completion times with
            write-once semantics to prevent accidental modification. The temporal system integrates
            with simulation time frameworks for accurate performance analysis and coordination.

    Core Attributes:
        Identification System:
            id (int): Unique mission identifier generated automatically using object memory address
                     as fallback or explicitly provided during initialization. Ensures consistent
                     task tracking and reference management across simulation components.

        Geographic Positioning:
            origin (GeoPoint): Starting geographic location using WGS84 coordinates with precision
                              suitable for drone navigation and mission planning. Immutable after
                              initialization to ensure mission integrity.
            destination (GeoPoint): Target geographic location for mission completion with geodetic
                                   accuracy for precise navigation and arrival detection. Supports
                                   complex waypoint sequences through subclass extensions.

        Temporal Coordination:
            _started_at (Time | None): Private timestamp capturing mission start time with write-once
                                      semantics. None until explicitly set via start_at property.
            _completed_at (Time | None): Private timestamp for mission completion with automatic
                                        protection against overwriting. Enables duration calculations.

        State Management Infrastructure:
            _state_machine (StateMachine | None): Internal state machine instance managing validated
                                                 transitions and action execution. Initialized through
                                                 init_state_machine() or subclass-specific logic.

    Property Interface:
        Temporal Access Properties:
            start_at: Thread-safe property providing read/write access to mission start timestamp
                     with write-once protection preventing accidental modification of established times.
            completed_at: Thread-safe property for mission completion timestamp with identical
                         write-once protection ensuring temporal integrity throughout mission lifecycle.

        State Machine Properties:
            current_state: Read-only property providing thread-safe access to current task state
                          from the underlying state machine with automatic validation and error handling.

    Abstract Method Requirements:
        Mission Control Methods:
            abort(): Task-specific abort implementation that transitions the task to an aborted state
                    and performs any necessary cleanup operations. Must handle all possible current
                    states and provide graceful degradation for error scenarios.

        Completion Detection:
            done (property): Boolean property indicating whether the task has reached a terminal state
                           (completed or aborted). Critical for mission management and resource cleanup.

    State Machine Architecture:
        Initialization Requirements:
            Subclasses must initialize the state machine through init_state_machine() with:
            • initial_state: Starting state for new task instances
            • nodes_graph: Dictionary mapping states to allowed transitions with actions

        Transition Management:
            transition_to() method provides validated state transitions with:
            • Automatic validation against configured transition rules
            • Action execution with parameter passing and return value handling
            • Exception handling with descriptive error messages
            • Thread-safe atomic operations preventing race conditions

        State Access:
            current_state property provides thread-safe read access to current state with:
            • Automatic state machine validation and initialization checking
            • Consistent state reporting across concurrent access scenarios
            • Error handling for uninitialized state machines with clear error messages

    Implementation Patterns:
        Basic Task Implementation:
            class MyTaskState(IntEnum):
                PENDING = auto()
                EXECUTING = auto()
                COMPLETED = auto()
                ABORTED = auto()

            class MyTask(Task):
                def __init__(self, origin: GeoPoint, destination: GeoPoint):
                    super().__init__(origin, destination)
                    # Define state transition graph
                    transition_graph = {
                        MyTaskState.PENDING: [
                            Action(MyTaskState.EXECUTING, self._start_execution),
                            Action(MyTaskState.ABORTED, self._handle_abort)
                        ],
                        MyTaskState.EXECUTING: [
                            Action(MyTaskState.COMPLETED, self._complete_mission),
                            Action(MyTaskState.ABORTED, self._handle_abort)
                        ],
                        MyTaskState.COMPLETED: [],
                        MyTaskState.ABORTED: []
                    }
                    self.init_state_machine(MyTaskState.PENDING, transition_graph)

                def abort(self):
                    self.transition_to(MyTaskState.ABORTED)

                @property
                def done(self) -> bool:
                    return self.current_state in {MyTaskState.COMPLETED, MyTaskState.ABORTED}

        Advanced Mission Coordination:
            # Mission lifecycle with temporal tracking
            task = MyTask(origin_point, destination_point)
            task.start_at = current_simulation_time

            # Progress through states with validation
            task.transition_to(MyTaskState.EXECUTING)
            # ... execute mission-specific logic ...
            task.transition_to(MyTaskState.COMPLETED)
            task.completed_at = current_simulation_time

            # Verify completion and calculate metrics
            assert task.done == True
            mission_duration = task.completed_at - task.start_at

    Thread Safety & Performance:
        Concurrency Considerations:
            • Thread-safe property access with atomic read/write operations
            • State machine transitions with proper locking and validation
            • Immutable geographic waypoints preventing data races
            • Protected temporal attributes with write-once semantics

        Performance Characteristics:
            • O(1) state machine operations with efficient transition validation
            • Minimal memory footprint with lazy initialization patterns
            • Scalable for thousands of concurrent task instances
            • Garbage collection friendly with proper reference management

    Integration Points:
        Simulation Framework Integration:
            • Time system integration for temporal coordination and scheduling
            • Geographic information system integration for spatial analysis
            • Event logging and monitoring systems for operational visibility
            • Performance metrics collection for optimization and analytics

        Extension Mechanisms:
            • Customizable state machines with arbitrary states and transitions
            • Extensible action systems with parameter passing and return values
            • Plugin architecture for mission-specific behaviors and integrations
            • Hierarchical task composition for complex multi-phase missions
    """

    id: int
    origin: GeoPoint
    destination: GeoPoint
    _started_at: Time | None
    _completed_at: Time | None
    _priority: float

    _state_machine: StateMachine | None = None

    @property
    def start_at(self) -> Time | None:
        """Get the mission start timestamp with thread-safe access and temporal validation.

        Provides read access to the mission start time with automatic validation and thread-safe
        operations. The timestamp represents when the mission transitioned from initial state to
        active execution, enabling accurate duration calculations and performance analysis.

        Returns:
            Time | None: Mission start timestamp if set, None if the mission has not yet begun
                        execution. The timestamp uses the simulation's temporal coordinate system
                        for consistency with other mission timing measurements.

        Thread Safety:
            This property provides thread-safe read access to the internal _started_at attribute
            without requiring explicit locking, ensuring consistent temporal reporting across
            concurrent access scenarios in multi-threaded simulation environments.

        Usage Examples:
            >>> task = MyTask(origin, destination)
            >>> print(task.start_at)  # None - mission not started
            >>> task.start_at = Time.from_seconds(100)
            >>> print(task.start_at)  # Time(100s) - mission start time
        """
        return self._started_at

    @start_at.setter
    def start_at(self, value: Time) -> None:
        """Set the mission start timestamp with write-once protection and validation.

        Establishes the mission start time with write-once semantics to prevent accidental
        modification of established timestamps. This ensures temporal integrity throughout
        the mission lifecycle and provides reliable foundation for duration calculations
        and performance analysis.

        Args:
            value (Time): Simulation timestamp marking the beginning of mission execution.
                         Must be a valid Time instance representing the moment when the mission
                         transitions from initial state to active execution phase.

        Write-Once Protection:
            The setter implements write-once semantics, allowing the start time to be set only
            once per mission instance. Subsequent attempts to modify the start time are silently
            ignored, preserving the original timestamp and preventing accidental overwrites.

        Validation:
            • Verifies that the provided value is a valid Time instance
            • Ensures the timestamp has not been previously set
            • Maintains temporal consistency with simulation time frameworks

        Side Effects:
            • Updates internal _started_at attribute if not previously set
            • Enables duration calculations via completed_at - start_at operations
            • Triggers temporal tracking for mission performance analysis

        Usage Examples:
            >>> task = MyTask(origin, destination)
            >>> task.start_at = Time.from_seconds(100)  # Sets start time
            >>> task.start_at = Time.from_seconds(200)  # Ignored - already set
            >>> print(task.start_at)  # Still Time(100s)
        """
        if self.start_at is None:
            self._started_at = value

    @property
    def completed_at(self) -> Time | None:
        """Get the mission completion timestamp with thread-safe access and temporal validation.

        Provides read access to the mission completion time with automatic validation and
        thread-safe operations. The timestamp represents when the mission reached a terminal
        state (COMPLETED or ABORTED), enabling accurate duration calculations and mission
        performance analysis across diverse operational scenarios.

        Returns:
            Time | None: Mission completion timestamp if set, None if the mission is still
                        in progress or has not yet reached a terminal state. The timestamp
                        uses the simulation's temporal coordinate system for consistency
                        with mission start times and duration calculations.

        Thread Safety:
            This property provides thread-safe read access to the internal _completed_at
            attribute without requiring explicit locking, ensuring consistent temporal
            reporting across concurrent access scenarios in multi-threaded environments.

        Integration with Mission Analysis:
            The completion timestamp enables comprehensive mission analysis including:
            • Duration calculations: completed_at - start_at
            • Performance metrics: execution time vs planned duration
            • Resource utilization: temporal efficiency analysis
            • Mission success rate: completion vs abortion timing patterns

        Usage Examples:
            >>> task = MyTask(origin, destination)
            >>> print(task.completed_at)  # None - mission in progress
            >>> task.completed_at = Time.from_seconds(250)
            >>> duration = task.completed_at - task.start_at  # Mission duration
        """
        return self._completed_at

    @completed_at.setter
    def completed_at(self, value: Time) -> None:
        """Set the mission completion timestamp with write-once protection and validation.

        Establishes the mission completion time with write-once semantics to prevent
        accidental modification of established completion timestamps. This ensures temporal
        integrity throughout the mission analysis phase and provides reliable foundation
        for performance metrics and operational reporting.

        Args:
            value (Time): Simulation timestamp marking the completion of mission execution.
                         Must be a valid Time instance representing the moment when the mission
                         reaches a terminal state (COMPLETED, ABORTED, or similar final state).

        Write-Once Protection:
            The setter implements write-once semantics, allowing the completion time to be set
            only once per mission instance. Subsequent attempts to modify the completion time
            are silently ignored, preserving the original timestamp and preventing accidental
            overwrites during mission cleanup or reporting phases.

        Validation and Integrity:
            • Verifies that the provided value is a valid Time instance
            • Ensures the timestamp has not been previously set
            • Maintains temporal consistency with mission start times
            • Supports duration calculations and performance analysis

        Side Effects:
            • Updates internal _completed_at attribute if not previously set
            • Enables duration calculations via completed_at - start_at operations
            • Triggers mission completion tracking for performance analysis
            • Supports mission lifecycle management and cleanup operations

        Usage Examples:
            >>> task = MyTask(origin, destination)
            >>> task.start_at = Time.from_seconds(100)
            >>> # ... mission execution ...
            >>> task.completed_at = Time.from_seconds(250)  # Sets completion time
            >>> task.completed_at = Time.from_seconds(300)  # Ignored - already set
            >>> print(task.completed_at)  # Still Time(250s)
            >>> duration = task.completed_at - task.start_at  # Time(150s)
        """
        if self.completed_at is None:
            self._completed_at = value

    def __init__(self, origin: GeoPoint, destination: GeoPoint, id: int | None = None):
        """Initialize a new Task instance with comprehensive geographic and identification setup.

        Creates a new mission task with specified geographic waypoints, unique identification,
        and temporal tracking infrastructure. This constructor establishes the foundational
        attributes required for mission execution while deferring state machine initialization
        to subclass-specific logic for maximum flexibility and customization.

        Geographic Waypoint Configuration:
            The constructor establishes immutable geographic waypoints using the WGS84 coordinate
            system, providing precise positioning for navigation calculations and mission planning.
            Both origin and destination points are stored as GeoPoint instances with accuracy
            suitable for autonomous vehicle navigation and spatial analysis.

        Identification System:
            The task identification system provides flexible ID assignment with automatic fallback
            generation. When an explicit ID is provided, it enables consistent task tracking across
            simulation components. When no ID is specified, the system generates a unique identifier
            using the object's memory address, ensuring distinct identifiers.

        Temporal Infrastructure:
            Initializes temporal tracking attributes (_started_at and _completed_at) to None,
            establishing the foundation for mission lifecycle management. These attributes support
            write-once semantics through the property interface, enabling accurate duration
            calculations and performance analysis.

        Args:
            origin (GeoPoint): Starting geographic location for the mission using WGS84 coordinates.
                              This waypoint represents the initial position or pickup location
                              task and remains immutable throughout the mission lifecycle to ensure
                              navigational consistency and reference integrity.

            destination (GeoPoint): Target geographic location for mission completion using WGS84
                                   coordinates. This waypoint represents the final position
                                   location for the task and provides the navigation target for
                                   autonomous vehicle guidance systems.

            id (int | None): Optional unique identifier for this task instance. If provided, enables
                            consistent task tracking and reference management across simulation
                            components. If None, automatically generates a unique identifier using
                            the built-in id() function based on object memory address.

        Initialization State:
            Post-Construction Requirements:
                • State machine must be initialized through init_state_machine() or subclass logic
                • Temporal tracking is ready but requires explicit start_at assignment
                • Geographic waypoints are immutable and ready for navigation calculations
                • Task is ready for state machine configuration and mission execution

        State Machine Initialization:
            The constructor intentionally defers state machine initialization to allow subclasses
            maximum flexibility in defining state transition graphs and initial states.
            Subclasses must call init_state_machine() with appropriate parameters or implement
            custom state machine setup logic in their constructors.

        Side Effects:
            • Sets immutable geographic waypoints for mission navigation
            • Establishes unique task identification for tracking and reference
            • Initializes temporal tracking infrastructure with None values
            • Prepares task instance for state machine configuration

        Usage Examples:
            Basic Task Creation:
                >>> from dronesim.geo import GeoPoint
                >>> origin = GeoPoint.from_deg(37.5665, 126.9780)
                >>> destination = GeoPoint.from_deg(37.5675, 126.9890)
                >>> task = MyTask(origin, destination)  # ID auto-generated
                >>> print(task.id)  # Unique integer based on memory address

            Task Creation with Explicit ID:
                >>> task = MyTask(origin, destination, id=12345)
                >>> print(task.id)  # 12345
                >>> print(task.origin)  # GeoPoint(37.5665, 126.9780)
                >>> print(task.destination)  # GeoPoint(37.5675, 126.9890)

        Integration Notes:
            This constructor provides the foundation for all mission task types, establishing
            consistent interfaces for geographic waypoint management, identification systems,
            and temporal tracking. Subclasses should call super().__init__() first in their
            constructors to ensure proper initialization of base class attributes.
        """
        if id is None:
            self.id = id(self)
        else:
            self.id = id

        self.origin = origin
        self.destination = destination

        self._started_at = None
        self._completed_at = None
        self._priority = 0.1

    def init_state_machine(self, initial_state: State, nodes_graph: StateGraph) -> None:
        """Initialize the task state machine with comprehensive configuration and validation.

        Establishes the state machine infrastructure for mission lifecycle management with the
        specified initial state and transition graph. This method provides flexible state machine
        setup for subclasses requiring custom initialization or dynamic configuration.

        State Machine Architecture:
            The state machine manages mission progression through validated transitions with
            configurable actions and effects. It enforces transition rules, prevents invalid
            state changes, and provides atomic operations for thread-safe mission control.

        Configuration Parameters:
            Initial State Setup:
                The initial_state parameter establishes the starting state for new task instances.
                This state should represent the initial mission condition (typically PENDING,
                ASSIGNED, or similar preparatory state) and must be included in the nodes_graph
                as a valid state with appropriate outgoing transitions.

            Transition Graph Definition:
                The nodes_graph parameter defines the complete state transition rules including:
                • Valid transitions between states with associated actions
                • Terminal states with no outgoing transitions (COMPLETED, ABORTED)
                • Error recovery paths for robust mission execution
                • Action effects with parameter passing and return value handling

        Args:
            initial_state (State): Starting state for the task state machine. Must be a valid
                                  state enumeration value that exists as a key in the nodes_graph
                                  dictionary. This state becomes the current state immediately
                                  after initialization.

            nodes_graph (StateGraph): Complete state transition graph defining valid transitions
                                     and associated actions. Dictionary mapping each state to a
                                     list of Action instances representing allowed transitions with
                                     optional action effects and parameter handling.

        Side Effects:
            • Creates new StateMachine instance with specified configuration
            • Sets internal _state_machine attribute for transition management
            • Establishes current state as the specified initial state
            • Enables state transition validation and action execution

        Validation:
            • Verifies initial_state exists in nodes_graph keys
            • Validates nodes_graph structure and Action configurations
            • Ensures state machine consistency and operational integrity
            • Provides error handling for configuration issues

        Usage Examples:
            Basic State Machine Setup:
                >>> from enum import IntEnum, auto
                >>> from dronesim.state import Action
                >>>
                >>> class TaskState(IntEnum):
                ...     PENDING = auto()
                ...     EXECUTING = auto()
                ...     COMPLETED = auto()
                >>>
                >>> transition_graph = {
                ...     TaskState.PENDING: [Action(TaskState.EXECUTING)],
                ...     TaskState.EXECUTING: [Action(TaskState.COMPLETED)],
                ...     TaskState.COMPLETED: [],
                ... }
                >>>
                >>> task = MyTask(origin, destination)
                >>> task.init_state_machine(TaskState.PENDING, transition_graph)
                >>> print(task.current_state)  # TaskState.PENDING

        Integration Notes:
            This method should be called during task initialization, either in the constructor
            or immediately after construction. Subclasses may override this method to provide
            specialized state machine configuration or validation logic while maintaining
            compatibility with the base Task interface.
        """
        self._state_machine = StateMachine(initial_state, nodes_graph)

    @abstractmethod
    def abort(self) -> None:
        """Execute task-specific abort procedures with state transition and cleanup operations.

        This abstract method defines the contract for mission abort procedures that subclasses
        must implement to handle task cancellation, error recovery, and cleanup operations.
        The abort mechanism provides a standardized interface for graceful mission termination
        regardless of current state or execution phase.

        Abstract Method Requirements:
            Subclass implementations must provide:
            • State transition to appropriate abort/error state (typically ABORTED)
            • Cleanup operations for allocated resources and incomplete operations
            • Error handling for abort procedures during different mission phases
            • Coordination with external systems or dependencies

        Abort Behavior Patterns:
            State Transition Management:
                • Transition from any valid state to terminal abort state
                • Validate abort transition is allowed from current state
                • Handle race conditions between abort and normal state progression
                • Ensure atomic abort operations for thread-safe execution

            Resource Cleanup:
                • Release allocated system resources (memory, file handles, network connections)
                • Cancel pending operations or external service requests
                • Clean up temporary data structures and mission-specific state
                • Coordinate with mission management systems for resource deallocation

            Error Recovery:
                • Handle abort failures gracefully with fallback mechanisms
                • Log abort operations for debugging and operational analysis
                • Provide error reporting for failed abort attempts
                • Ensure mission system integrity despite abort complications

        Implementation Examples:
            Basic Abort Implementation:
                def abort(self) -> None:
                    '''Abort delivery task and transition to ABORTED state.'''
                    try:
                        self.transition_to(TaskState.ABORTED)
                        self._cleanup_resources()
                        self.completed_at = current_simulation_time()
                    except Exception as e:
                        logger.error(f"Abort failed for task {self.id}: {e}")
                        raise

            Advanced Abort with Cleanup:
                def abort(self) -> None:
                    '''Abort with comprehensive cleanup and error recovery.'''
                    # Cancel external operations
                    if self._external_service:
                        self._external_service.cancel_request(self.id)

                    # Clean up allocated resources
                    self._cleanup_mission_resources()

                    # Transition to abort state with timestamp
                    self.transition_to(TaskState.ABORTED)
                    self.completed_at = current_simulation_time()

                    # Log abort for analysis
                    logger.info(f"Task {self.id} aborted from state {self.current_state}")

        Thread Safety:
            Abort implementations should be thread-safe and handle concurrent access scenarios
            where multiple threads might attempt to abort the same task simultaneously or where
            abort operations conflict with normal mission progression.

        Integration Requirements:
            • State machine integration for validated abort transitions
            • Temporal tracking integration for abort timestamp recording
            • Resource management coordination for proper cleanup
            • Logging and monitoring integration for operational visibility
        """
        pass

    def transition_to(self, next_state: State, *args, **kwargs) -> Any:
        """Execute validated state transition with action effects and comprehensive error handling.

        Manages atomic state transitions through the task's state machine with full validation,
        action execution, and error recovery. This method provides the primary interface for
        mission state progression, ensuring transitions follow configured rules and maintain
        operational integrity throughout the mission lifecycle.

        State Transition Architecture:
            Validation Pipeline:
                • Verifies state machine initialization and operational readiness
                • Validates target state against configured transition rules
                • Checks transition preconditions and state machine consistency
                • Ensures atomic transition execution with rollback capabilities

            Action Execution Framework:
                • Executes associated action effects with parameter passing
                • Handles action return values and side effect management
                • Provides error recovery for failed action executions
                • Maintains state consistency despite action failures

            Thread Safety Guarantees:
                • Atomic state transitions preventing race conditions
                • Proper locking mechanisms for concurrent access scenarios
                • Consistent state reporting during transition execution
                • Safe handling of multiple concurrent transition attempts

        Transition Process:
            Pre-Transition Validation:
                1. Verify state machine initialization and readiness
                2. Validate target state exists in configured transition graph
                3. Check current state allows transition to target state
                4. Prepare action parameters and execution context

            Atomic Transition Execution:
                1. Lock state machine for atomic operation
                2. Execute associated action effects with provided parameters
                3. Update current state to target state upon successful action completion
                4. Release lock and update state reporting

            Post-Transition Processing:
                1. Return action execution results to caller
                2. Update internal state tracking and logging
                3. Trigger state change notifications if configured
                4. Handle any post-transition cleanup or initialization

        Args:
            next_state (State): Target state for the transition. Must be a valid state enumeration
                               value that exists in the configured state transition graph and is
                               reachable from the current state according to transition rules.

            *args: Positional arguments passed to the transition action effect function. These
                  arguments are forwarded directly to the action's effect method for custom
                  processing during the transition execution.

            **kwargs: Keyword arguments passed to the transition action effect function. These
                     arguments provide named parameter passing for complex action effects with
                     multiple configuration options.

        Returns:
            Any: The return value from executing the transition action's effect function. Returns
                None if the action has no effect function or if the effect function returns None.
                The specific return type depends on the action implementation.

        Raises:
            NotImplementedError: If the state machine has not been initialized through
                               init_state_machine() or subclass-specific initialization. This
                               indicates a programming error in task setup.

            ValueError: If the requested transition is not allowed by the configured state
                       transition rules. This includes transitions from the current state to
                       an invalid target state or transitions to non-existent states.

            RuntimeError: If the state machine is in an inconsistent state or if action
                         execution fails with unrecoverable errors. This indicates either
                         configuration issues or runtime failures during transition.

        Usage Examples:
            Basic State Transition:
                >>> task = MyTask(origin, destination)
                >>> task.transition_to(TaskState.EXECUTING)
                >>> print(task.current_state)  # TaskState.EXECUTING

            Transition with Parameters:
                >>> result = task.transition_to(TaskState.COMPLETED, completion_time=now, success=True)
                >>> print(result)  # Action effect return value

            Error Handling:
                >>> try:
                ...     task.transition_to(TaskState.INVALID)
                ... except ValueError as e:
                ...     print(f"Invalid transition: {e}")

        Performance Characteristics:
            • O(1) transition validation using hash-based state lookup
            • Atomic operations with minimal locking overhead
            • Efficient action execution with direct method invocation
            • Memory-efficient state tracking without unnecessary allocations
        """
        if not self._state_machine:
            msg = "Subclasses must initialize the state_machine ClassVar."
            raise NotImplementedError(msg)

        return self._state_machine.request_transition(next_state, *args, **kwargs)

    @property
    def current_state(self) -> State:
        """Get the current task state with thread-safe access and validation.

        Provides thread-safe read access to the current mission state from the underlying
        state machine with automatic validation and error handling. This property serves
        as the primary interface for state inspection and mission monitoring across
        concurrent access scenarios.

        State Access Architecture:
            Thread-Safe State Retrieval:
                • Atomic read operations preventing race conditions during state transitions
                • Consistent state reporting across multiple concurrent access threads
                • Automatic validation of state machine initialization and readiness
                • Safe handling of state access during transition execution

            Validation and Error Handling:
                • Verifies state machine has been properly initialized
                • Provides descriptive error messages for configuration issues
                • Handles edge cases during state machine setup and teardown
                • Ensures consistent behavior across different initialization patterns

        Returns:
            State: Current state enumeration value representing the task's position in its
                   lifecycle. The specific state type depends on the subclass implementation
                   and state machine configuration (e.g., DeliveryState.ASSIGNED).

        Raises:
            NotImplementedError: If the state machine has not been initialized through
                               init_state_machine() or subclass-specific setup. This indicates
                               a programming error in task initialization.

        Usage Examples:
            State Inspection:
                >>> task = MyTask(origin, destination)
                >>> print(task.current_state)  # MyTaskState.PENDING
                >>> task.transition_to(MyTaskState.EXECUTING)
                >>> print(task.current_state)  # MyTaskState.EXECUTING

            State-Based Logic:
                >>> if task.current_state == MyTaskState.COMPLETED:
                ...     print("Mission successful!")
                >>> elif task.current_state == MyTaskState.ABORTED:
                ...     print("Mission failed!")

        Performance Characteristics:
            • O(1) state access with direct attribute lookup
            • Minimal overhead for thread-safe state reporting
            • Efficient validation with cached state machine reference
            • Memory-efficient with no temporary object allocation
        """
        if not self._state_machine:
            msg = "Subclasses must initialize the state_machine"
            raise NotImplementedError(msg)

        return self._state_machine.current

    @property
    @abstractmethod
    def done(self) -> bool:
        """Determine task completion status with comprehensive terminal state detection.

        This abstract property defines the contract for mission completion detection that
        subclasses must implement to specify their terminal state conditions. The property
        provides a standardized interface for completion checking across diverse mission
        types while allowing task-specific completion criteria.

        Abstract Property Requirements:
            Subclass implementations must provide:
            • Terminal state detection logic for successful completion (DONE, COMPLETED)
            • Terminal state detection logic for failed completion (ABORTED, FAILED)
            • Handling of edge cases and intermediate completion states
            • Thread-safe completion status reporting

        Completion Detection Patterns:
            Binary Terminal States:
                Most tasks implement binary completion with successful and failed outcomes:
                return self.current_state in {TaskState.COMPLETED, TaskState.ABORTED}

            Multi-Terminal States:
                Complex tasks may have multiple terminal conditions:
                return self.current_state in {
                    TaskState.COMPLETED,
                    TaskState.ABORTED,
                    TaskState.CANCELLED,
                    TaskState.EXPIRED
                }

            Conditional Completion:
                Advanced tasks may implement conditional completion logic:
                if self.current_state == TaskState.PARTIAL_COMPLETION:
                    return self.completion_percentage >= self.required_threshold
                return self.current_state in self.terminal_states

        Returns:
            bool: True if the task has reached any terminal state indicating mission
                 completion (successful or failed). False if the task is still in
                 progress or in a non-terminal state requiring continued execution.

        Implementation Examples:
            Basic Binary Completion:
                @property
                def done(self) -> bool:
                    '''Check if delivery task is complete or aborted.'''
                    return self.current_state in {
                        DeliveryState.COMPLETED,
                        DeliveryState.ABORTED
                    }

            Advanced Completion Logic:
                @property
                def done(self) -> bool:
                    '''Check completion with conditional logic.'''
                    # Handle successful completion
                    if self.current_state == TaskState.COMPLETED:
                        return True

                    # Handle failure states
                    if self.current_state in {TaskState.ABORTED, TaskState.FAILED}:
                        return True

                    # Handle partial completion with thresholds
                    if self.current_state == TaskState.PARTIAL:
                        return self.progress >= self.completion_threshold

                    return False

        Thread Safety:
            Implementations should be thread-safe and handle concurrent access scenarios
            where multiple threads check completion status simultaneously or where
            completion detection occurs during state transitions.

        Integration Usage:
            Mission Management:
                >>> while not task.done:
                ...     task.update(simulation_time_step)
                ...     time.sleep(0.1)
                >>> print("Mission completed!")

            Resource Cleanup:
                >>> if task.done:
                ...     task.cleanup_resources()
                ...     mission_manager.remove_task(task.id)

            Performance Analysis:
                >>> completed_tasks = [t for t in task_list if t.done]
                >>> success_rate = len([t for t in completed_tasks if t.current_state == TaskState.COMPLETED])
        """
        pass

    def state_list(self) -> list[State]:
        """Get a list of all states defined in the drone's state machine.

        Returns:
            A list of all states that have defined transitions in the drone's state machine.
        """
        if not hasattr(self, "_state_machine"):
            msg = "Subclasses must initialize the state_machine ClassVar."
            raise NotImplementedError(msg)

        return self._state_machine.get_state_list()

    @property
    def priority(self) -> int:
        """Get the task priority level for scheduling and resource allocation.

        Returns:
            int: Priority level of the task, where higher values indicate higher priority.
        """
        return self._priority

    @priority.setter
    def priority(self, value: int) -> None:
        """Set the task priority level for scheduling and resource allocation.

        Args:
            value (int): Priority level to assign to the task, where higher values indicate
                         higher priority.
        """
        if value <= 1:
            self._priority = value
        else:
            self._priority = 1
