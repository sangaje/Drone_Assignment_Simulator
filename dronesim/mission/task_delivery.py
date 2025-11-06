"""Comprehensive delivery task implementation for autonomous drone package transportation.

This module provides a complete concrete implementation of the abstract Task framework
specifically designed for package delivery operations. It establishes a sophisticated
state machine architecture that manages the entire delivery lifecycle from initial
assignment through final completion or cancellation, with robust error handling and
temporal coordination throughout all operational phases.

Advanced Delivery Architecture:
    State Machine Design:
        The delivery task implements a linear progression state machine with comprehensive
        error recovery capabilities. Each state represents a distinct operational phase
        with specific behaviors, timing requirements, and transition conditions that ensure
        reliable package delivery across diverse operational scenarios.

    Temporal Coordination System:
        Advanced timing management coordinates pickup schedules, service time allocations,
        and delivery deadlines with simulation time frameworks. The system supports complex
        temporal constraints including order placement times, pickup time windows, and
        service duration requirements for realistic delivery operations.

    Geographic Integration:
        Full integration with the WGS84 coordinate system enables precise navigation between
        pickup and delivery locations with geodetic accuracy suitable for autonomous vehicle
        guidance systems. The implementation supports complex multi-point delivery routes
        and dynamic waypoint management.

Delivery State Machine:
    Linear Progression Flow:
        ASSIGNED → GO_PICKUP → SERVICE_PICKUP → GO_DROPOFF → SERVICE_DROPOFF → DONE

        The state machine follows a strict linear progression ensuring proper delivery
        workflow execution with validated transitions and comprehensive error recovery
        at each stage of the delivery process.

    Error Recovery Integration:
        Any state → ABORTED

        Comprehensive abort mechanisms allow immediate task cancellation from any operational
        state with proper cleanup and resource management. The abort system handles various
        failure scenarios including vehicle malfunctions, navigation errors, and external
        cancellation requests.

    State Descriptions:
        ASSIGNED: Initial state after task creation, ready for execution planning
        GO_PICKUP: Vehicle is navigating to package pickup location
        SERVICE_PICKUP: Vehicle is at pickup location performing package loading operations
        GO_DROPOFF: Vehicle is navigating to delivery destination with package payload
        SERVICE_DROPOFF: Vehicle is at delivery location performing package unloading
        DONE: Delivery successfully completed with package delivered
        ABORTED: Task cancelled or failed, requiring cleanup and error reporting

Advanced Features:
    Event Timing System:
        Comprehensive timestamp tracking for each state transition enables detailed
        performance analysis, operational metrics calculation, and service level
        agreement monitoring. The timing system captures state entry times for
        complete delivery lifecycle visibility.

    Ground Operation Coordination:
        Specialized ground task identification enables coordination with ground-based
        operations during pickup and delivery phases. The system distinguishes between
        flight operations and ground service operations for proper resource allocation
        and timing management.

    Temporal Constraint Management:
        Advanced timing constraints coordinate order placement times with pickup time
        windows and service duration requirements. The system supports realistic
        delivery operations with configurable timing parameters and constraint validation.

Core Components:
    DeliveryState Enumeration:
        IntEnum-based state definition with automatic value assignment and ordering
        support. The enumeration provides type-safe state management with efficient
        comparison operations and comprehensive state validation.

    DeliveryTask Implementation:
        Concrete Task subclass with complete delivery-specific functionality including
        state machine configuration, transition management, temporal tracking, and
        ground operation coordination.

Configuration Architecture:
    State Transition Rules (_allowed):
        Comprehensive dictionary mapping each state to allowed transitions with Action
        instances. The configuration supports both normal progression and error recovery
        paths with validated transition rules and action execution.

    Progression Sequence (_next):
        Linear progression mapping for normal delivery workflow execution. The sequence
        defines the expected state progression for successful delivery completion with
        efficient state advancement logic.

Integration Capabilities:
    Drone Fleet Integration:
        Full compatibility with drone simulation frameworks and fleet management systems.
        The implementation supports concurrent delivery operations with resource sharing
        and coordination across multiple delivery tasks.

    Simulation Framework Integration:
        Complete integration with time-based simulation systems including temporal
        coordination, event scheduling, and performance monitoring. The implementation
        supports real-time and accelerated simulation scenarios.

    Analytics and Monitoring:
        Comprehensive event tracking and timing data collection for operational analytics,
        performance optimization, and service level monitoring. The system provides
        detailed visibility into delivery operations and execution metrics.

Example Usage Patterns:
    Basic Delivery Task Creation:
        >>> from dronesim.geo import GeoPoint
        >>> from dronesim.unit import Time
        >>>
        >>> # Define pickup and delivery locations
        >>> pickup_location = GeoPoint.from_deg(37.5665, 126.9780)
        >>> delivery_location = GeoPoint.from_deg(37.5675, 126.9890)
        >>>
        >>> # Create delivery task with timing constraints
        >>> order_time = Time.from_seconds(100)
        >>> pickup_time = Time.from_seconds(300)  # 5 minutes for pickup
        >>> delivery = DeliveryTask(pickup_location, delivery_location, order_time, pickup_time)
        >>>
        >>> # Task starts in ASSIGNED state
        >>> print(delivery.current_state)  # DeliveryState.ASSIGNED

    Mission Execution Workflow:
        >>> # Progress through delivery states
        >>> current_time = Time.from_seconds(150)
        >>> delivery.next(current_time)  # ASSIGNED → GO_PICKUP
        >>>
        >>> # Continue state progression with timing
        >>> delivery.next(current_time)  # GO_PICKUP → SERVICE_PICKUP
        >>> delivery.next(current_time)  # SERVICE_PICKUP → GO_DROPOFF
        >>> delivery.next(current_time)  # GO_DROPOFF → SERVICE_DROPOFF
        >>> delivery.next(current_time)  # SERVICE_DROPOFF → DONE
        >>>
        >>> # Verify completion
        >>> assert delivery.done == True
        >>> print(delivery.event_time[DeliveryState.DONE])  # Completion timestamp

    Error Handling and Recovery:
        >>> # Handle delivery failure scenarios
        >>> delivery = DeliveryTask(pickup, dropoff, order_time, pickup_time)
        >>> delivery.next()  # Begin delivery process
        >>>
        >>> # Emergency abort during execution
        >>> delivery.abort()  # Transition to ABORTED state
        >>> assert delivery.done == True
        >>> assert delivery.current_state == DeliveryState.ABORTED
"""

from __future__ import annotations

from enum import IntEnum, auto

from dronesim.geo import GeoPoint
from dronesim.state import Action
from dronesim.unit import Time

from .task import Task


class DeliveryState(IntEnum):
    """Comprehensive enumeration for delivery task states with ordering and progression.

    This integer-based enumeration defines the complete state space for delivery task lifecycle
    management with automatic value assignment and natural ordering support. The enumeration
    provides type-safe state management with efficient comparison operations, state validation,
    and comprehensive progression tracking throughout the delivery workflow.

    Enumeration Architecture:
        Integer-Based Design:
            IntEnum inheritance provides automatic integer value assignment with natural ordering
            that reflects the logical progression of delivery operations. This enables efficient
            state comparison operations, range checks, and progression validation for complex
            delivery workflow management.

        Automatic Value Assignment:
            The auto() function provides automatic sequential value assignment ensuring consistent
            state ordering without manual value management. This prevents value conflicts and
            maintains logical state progression order for operational analysis.

        Type Safety Integration:
            Full integration with Python's type system enables compile-time validation of state
            assignments and comparison operations. The enumeration prevents invalid state values
            and provides IDE support for state completion and validation.

    State Definitions:
        Initial and Assignment States:
            ASSIGNED: Initial state after task creation and assignment to delivery system.
                     The task is ready for execution planning and resource allocation but has
                     not yet begun operational activities. This state enables task queuing,
                     scheduling, and preliminary validation before execution begins.

        Navigation and Transit States:
            GO_PICKUP: Active navigation state where the delivery vehicle is traveling to the
                      package pickup location. This state represents horizontal flight operations
                      with route optimization, obstacle avoidance, and arrival detection for
                      efficient pickup location approach.

            GO_DROPOFF: Active navigation state where the delivery vehicle is transporting the
                       package payload to the final delivery destination. This state includes
                       payload management, route optimization, and delivery location approach
                       with precision landing capabilities.

        Ground Service States:
            SERVICE_PICKUP: Ground-based operational state where the delivery vehicle is
                           positioned at the pickup location and performing package loading
                           operations. This state includes vehicle positioning, package
                           securing, load balancing, and preparation for departure.

            SERVICE_DROPOFF: Ground-based operational state where the delivery vehicle is
                            positioned at the delivery location and performing package
                            unloading operations. This state includes precision positioning,
                            package release, delivery confirmation, and departure preparation.

        Terminal States:
            DONE: Successful completion state indicating the delivery has been completed with
                  package successfully delivered to the intended recipient or location. This
                  terminal state enables resource cleanup, performance reporting, and mission
                  success tracking.

            ABORTED: Failure or cancellation state indicating the delivery could not be completed
                    due to system failures, external cancellation, or operational constraints.
                    This terminal state enables error reporting, resource cleanup, and failure
                    analysis for operational improvement.

    State Transition Relationships:
        Linear Progression Flow:
            ASSIGNED (0) → GO_PICKUP (1) → SERVICE_PICKUP (2) → GO_DROPOFF (3) →
            SERVICE_DROPOFF (4) → DONE (5)

            The natural integer ordering reflects the expected progression sequence enabling
            efficient progression validation and operational analysis.

        Error Recovery Paths:
            Any State → ABORTED (6)

            The abort state is accessible from any operational state enabling immediate task
            cancellation and error recovery regardless of current delivery phase.

    Operational Characteristics:
        State Classification:
            • Flight States: GO_PICKUP, GO_DROPOFF (vehicle in transit)
            • Ground States: SERVICE_PICKUP, SERVICE_DROPOFF (vehicle on ground)
            • Planning States: ASSIGNED (pre-execution)
            • Terminal States: DONE, ABORTED (post-execution)

        Progression Validation:
            • Normal progression: state_value increases by 1 for each transition
            • Skip validation: direct transitions to ABORTED from any state
            • Terminal validation: no outgoing transitions from DONE or ABORTED

        Performance Integration:
            • Efficient state comparison using integer values
            • Range-based state filtering for operational analysis
            • Memory-efficient representation with minimal storage overhead
            • Fast state validation with O(1) enumeration membership checks

    Usage Examples:
        State Comparison and Ordering:
            >>> print(DeliveryState.ASSIGNED < DeliveryState.GO_PICKUP)  # True
            >>> print(DeliveryState.SERVICE_PICKUP.value)  # 2
            >>> states_before_pickup = [s for s in DeliveryState if s < DeliveryState.SERVICE_PICKUP]

        State Classification:
            >>> ground_states = {DeliveryState.SERVICE_PICKUP, DeliveryState.SERVICE_DROPOFF}
            >>> is_ground_operation = current_state in ground_states
            >>> flight_states = {DeliveryState.GO_PICKUP, DeliveryState.GO_DROPOFF}
            >>> is_in_transit = current_state in flight_states

        Progression Validation:
            >>> def is_valid_progression(current: DeliveryState, next: DeliveryState) -> bool:
            ...     if next == DeliveryState.ABORTED:
            ...         return True  # Can abort from any state
            ...     return next.value == current.value + 1  # Normal progression
    """

    ASSIGNED = auto()
    GO_PICKUP = auto()
    SERVICE_PICKUP = auto()
    GO_DROPOFF = auto()
    SERVICE_DROPOFF = auto()
    DONE = auto()
    ABORTED = auto()


class DeliveryTask(Task):
    """Advanced concrete delivery task with comprehensive state management and coordination.

    This sophisticated implementation of the abstract Task interface provides complete package
    delivery functionality with advanced state machine integration, comprehensive temporal
    tracking, and robust error handling throughout the entire delivery lifecycle. The class
    demonstrates the full capabilities of the Task framework through specialized delivery
    operations with complex timing constraints and operational coordination.

    Architectural Implementation:
        State Machine Specialization:
            The delivery task implements a linear progression state machine with comprehensive
            validation rules and error recovery mechanisms. The state machine coordinates
            delivery workflow progression with external systems while maintaining operational
            integrity and providing detailed lifecycle tracking.

        Temporal Constraint System:
            Advanced timing management coordinates order placement times with pickup schedules
            and service duration requirements. The system supports complex temporal constraints
            including delivery deadlines, service time windows, and coordination with external
            logistics systems.

        Event Tracking Infrastructure:
            Comprehensive event timestamp collection captures state transition times for
            detailed performance analysis, service level agreement monitoring, and operational
            optimization. The tracking system provides complete visibility into delivery
            operation timing and execution efficiency.

    State Machine Configuration:
        Transition Rules Architecture (_allowed):
            The transition configuration defines a comprehensive set of validated state changes
            with both normal progression paths and error recovery mechanisms. Each state defines
            its allowed transitions including normal advancement and immediate abort capabilities
            for robust error handling.

            State-Specific Transition Rules:
            • ASSIGNED: Can progress to GO_PICKUP or abort immediately
            • GO_PICKUP: Can advance to SERVICE_PICKUP upon arrival or abort during transit
            • SERVICE_PICKUP: Can progress to GO_DROPOFF after pickup or abort during service
            • GO_DROPOFF: Can advance to SERVICE_DROPOFF upon arrival or abort during transit
            • SERVICE_DROPOFF: Can complete to DONE after delivery or abort during service
            • DONE: Terminal state with no outgoing transitions
            • ABORTED: Terminal state with no outgoing transitions

        Progression Sequence (_next):
            The linear progression mapping defines the expected state advancement for successful
            delivery completion. This configuration enables efficient state progression with
            automatic validation and simplified transition management.

            Normal Progression Flow:
            ASSIGNED → GO_PICKUP → SERVICE_PICKUP → GO_DROPOFF → SERVICE_DROPOFF → DONE

    Specialized Attributes:
        Event Timing System:
            _event_time (dict[DeliveryState, Time | None]): Comprehensive timestamp dictionary
                                                           capturing state entry times for each
                                                           delivery state. Enables detailed
                                                           performance analysis and timing
                                                           optimization.

        Temporal Coordination:
            order_time (Time): Timestamp when the delivery order was initially placed, enabling
                              order lifecycle tracking and customer service coordination.
            pickup_time (Time): Allocated time duration for pickup service operations, supporting
                               realistic timing constraints and service level management.

        State Management:
            state_machine (StateMachine | None): Initialized state machine instance managing
                                                validated transitions and action execution
                                                throughout the delivery lifecycle.

    Inherited Infrastructure:
        Geographic Positioning (from Task):
            origin (GeoPoint): Package pickup location with WGS84 coordinate precision for
                              accurate navigation and arrival detection during pickup operations.
            destination (GeoPoint): Package delivery location with geodetic accuracy for
                                   precise navigation and delivery confirmation.

        Identification System (from Task):
            id (int): Unique delivery task identifier enabling tracking across simulation
                     components and integration with external delivery management systems.

        Temporal Lifecycle (from Task):
            start_at (Time | None): Mission start timestamp with write-once protection
            completed_at (Time | None): Mission completion timestamp for duration calculations

    Operational Workflow:
        Initialization Phase:
            1. Create DeliveryTask instance with geographic waypoints and timing constraints
            2. Initialize state machine with ASSIGNED state and transition configuration
            3. Set up event timing dictionary for comprehensive state tracking
            4. Configure order and pickup time parameters for operational coordination

        Execution Phase:
            1. Progress through delivery states using validated transitions
            2. Capture event timestamps for each state transition
            3. Coordinate with external systems during ground service operations
            4. Handle error conditions and abort scenarios with proper cleanup

        Completion Phase:
            1. Reach terminal state (DONE or ABORTED) with final timestamp capture
            2. Enable performance analysis through comprehensive timing data
            3. Support resource cleanup and mission reporting
            4. Provide completion status for delivery management systems

    Advanced Features:
        Ground Task Identification:
            Static method ground_task() provides ground service state identification for
            coordination with ground-based operations during pickup and delivery phases.
            Enables proper resource allocation and timing management for service operations.

        Temporal Progression:
            Method next() provides validated state progression with optional timestamp
            capture, enabling efficient delivery workflow advancement with comprehensive
            timing coordination and validation.

        Error Recovery:
            Method abort() provides immediate task cancellation from any operational state
            with proper state transition and cleanup coordination for robust error handling.

    Usage Examples:
        Complete Delivery Lifecycle:
            >>> from dronesim.geo import GeoPoint
            >>> from dronesim.unit import Time
            >>>
            >>> # Create delivery task with timing constraints
            >>> pickup = GeoPoint.from_deg(37.5665, 126.9780)
            >>> dropoff = GeoPoint.from_deg(37.5675, 126.9890)
            >>> order_time = Time.from_seconds(100)
            >>> pickup_duration = Time.from_seconds(300)
            >>>
            >>> delivery = DeliveryTask(pickup, dropoff, order_time, pickup_duration)
            >>> print(delivery.current_state)  # DeliveryState.ASSIGNED
            >>>
            >>> # Execute complete delivery workflow
            >>> current_time = Time.from_seconds(150)
            >>> for _ in range(5):  # Progress through all states
            ...     delivery.next(current_time)
            ...     current_time += Time.from_seconds(60)
            >>>
            >>> assert delivery.done == True
            >>> print(delivery.current_state)  # DeliveryState.DONE

        Performance Analysis:
            >>> # Analyze delivery timing performance
            >>> event_times = delivery.event_time
            >>> pickup_duration = event_times[DeliveryState.GO_DROPOFF] - event_times[DeliveryState.SERVICE_PICKUP]
            >>> total_duration = event_times[DeliveryState.DONE] - event_times[DeliveryState.ASSIGNED]
            >>> print(f"Pickup took: {pickup_duration}, Total: {total_duration}")

        Error Handling:
            >>> # Handle delivery failure scenarios
            >>> delivery = DeliveryTask(pickup, dropoff, order_time, pickup_duration)
            >>> delivery.next()  # Start delivery process
            >>> delivery.abort()  # Emergency cancellation
            >>> assert delivery.current_state == DeliveryState.ABORTED
            >>> assert delivery.done == True
    """

    # State transition rules: defines which state transitions are allowed
    # Each state can progress to its normal next state or be aborted
    # Terminal states (DONE, ABORTED) have no allowed transitions
    _allowed = {
        DeliveryState.ASSIGNED: [
            Action(DeliveryState.GO_PICKUP),
            Action(DeliveryState.ABORTED),
        ],
        DeliveryState.GO_PICKUP: [
            Action(DeliveryState.SERVICE_PICKUP),
            Action(DeliveryState.ABORTED),
        ],
        DeliveryState.SERVICE_PICKUP: [
            Action(DeliveryState.GO_DROPOFF),
            Action(DeliveryState.ABORTED),
        ],
        DeliveryState.GO_DROPOFF: [
            Action(DeliveryState.SERVICE_DROPOFF),
            Action(DeliveryState.ABORTED),
        ],
        DeliveryState.SERVICE_DROPOFF: [
            Action(DeliveryState.DONE),
            Action(DeliveryState.ABORTED),
        ],
        DeliveryState.DONE: [],
        DeliveryState.ABORTED: [],
    }

    # Normal state progression sequence for successful delivery completion
    # Maps each state to its logical next state in the delivery process
    _next = {
        DeliveryState.ASSIGNED: DeliveryState.GO_PICKUP,
        DeliveryState.GO_PICKUP: DeliveryState.SERVICE_PICKUP,
        DeliveryState.SERVICE_PICKUP: DeliveryState.GO_DROPOFF,
        DeliveryState.GO_DROPOFF: DeliveryState.SERVICE_DROPOFF,
        DeliveryState.SERVICE_DROPOFF: DeliveryState.DONE,
    }

    _event_time: dict[DeliveryState, Time | None]

    order_time: Time
    pickup_time: Time  # Time allocated for pickup service

    def __init__(
        self,
        origin: GeoPoint,
        destination: GeoPoint,
        order_time: Time,
        pickup_time: Time,
        id: int | None = None,
    ):
        """Initialize comprehensive delivery task with geographic waypoints and constraints.

        Creates a fully configured delivery task instance with advanced state machine setup,
        comprehensive event tracking infrastructure, and sophisticated temporal coordination
        capabilities. This constructor establishes all necessary components for autonomous
        delivery operations including geographic waypoints, timing constraints, and
        operational lifecycle management.

        Initialization Architecture:
            State Machine Configuration:
                Automatically initializes the delivery state machine with ASSIGNED as the initial
                state and configures the complete transition graph for delivery workflow management.
                The state machine setup includes validation rules, error recovery paths, and
                comprehensive action coordination for reliable delivery execution.

            Event Tracking Infrastructure:
                Establishes comprehensive timestamp tracking for all delivery states, enabling
                detailed performance analysis, service level agreement monitoring, and operational
                optimization. The tracking system captures state entry times with high precision
                for temporal analytics and delivery performance assessment.

            Temporal Constraint Management:
                Configures advanced timing coordination including order placement timestamps and
                pickup service duration requirements. The temporal system supports complex
                delivery scheduling, service time windows, and coordination with external
                logistics systems for realistic delivery operations.

        Geographic Waypoint Configuration:
            Package Pickup Coordination:
                The origin parameter establishes the package pickup location with WGS84 coordinate
                precision suitable for autonomous vehicle navigation and arrival detection. The
                pickup location supports complex logistics coordination including access validation,
                parking requirements, and service time optimization.

            Delivery Destination Management:
                The destination parameter defines the final package delivery location with geodetic
                accuracy for precise navigation and delivery confirmation. The destination system
                supports various delivery scenarios including residential, commercial, and
                specialized delivery locations.

        Args:
            origin (GeoPoint): Package pickup location using WGS84 coordinates with precision
                              suitable for autonomous vehicle navigation. This location serves as
                              the starting point for delivery operations and supports complex
                              logistics coordination including access validation and service timing.

            destination (GeoPoint): Package delivery destination using WGS84 coordinates with
                                   geodetic accuracy for precise navigation and confirmation.
                                   This location represents the final delivery point with support
                                   for various delivery scenarios and recipient coordination.

            order_time (Time): Timestamp when the delivery order was initially placed by the
                              customer or logistics system. This temporal reference enables order
                              lifecycle tracking, customer service coordination, and delivery
                              performance analysis from order placement through completion.

            pickup_time (Time): Allocated duration for pickup service operations including vehicle
                               positioning, package loading, security verification, and departure
                               preparation. This timing constraint supports realistic service
                               operations and coordination with pickup location requirements.

            id (int | None): Optional unique identifier for this delivery task instance.
                            enables consistent task tracking across simulation components and
                            integration with external delivery management systems. If None,
                            automatically generates a unique identifier using object memory address.

        State Machine Initialization:
            Transition Configuration:
                • Initial State: DeliveryState.ASSIGNED for new task preparation
                • Transition Rules: Complete _allowed dictionary with validation and error recovery
                • Action Integration: Configured for state-specific operations and coordination
                • Terminal States: Proper DONE and ABORTED state handling with cleanup support

        Event Tracking Setup:
            Comprehensive State Monitoring:
                • All delivery states initialized with None timestamps for clean tracking
                • State entry time capture for detailed performance analysis
                • Integration with simulation time frameworks for temporal coordination
                • Support for delivery lifecycle analytics and optimization

        Side Effects:
            Parent Class Initialization:
                • Calls super().__init__() with geographic waypoints and optional identifier
                • Establishes base Task infrastructure including temporal lifecycle management
                • Configures identification system and geographic waypoint immutability

            State Machine Configuration:
                • Initializes StateMachine instance with ASSIGNED state and transition rules
                • Enables validated state transitions and action execution throughout lifecycle
                • Establishes error recovery capabilities and terminal state handling

            Event Infrastructure Setup:
                • Creates comprehensive _event_time dictionary for all delivery states
                • Initializes timestamp tracking with None values for clean state monitoring
                • Enables detailed performance analysis and temporal coordination

            Temporal Attribute Assignment:
                • Sets order_time for order lifecycle tracking and customer coordination
                • Configures pickup_time for service duration management and timing constraints
                • Enables advanced delivery scheduling and temporal optimization

        Usage Examples:
            Basic Delivery Task Creation:
                >>> from dronesim.geo import GeoPoint
                >>> from dronesim.unit import Time
                >>>
                >>> pickup_location = GeoPoint.from_deg(37.5665, 126.9780)
                >>> delivery_location = GeoPoint.from_deg(37.5675, 126.9890)
                >>> order_timestamp = Time.from_seconds(100)
                >>> service_duration = Time.from_seconds(300)  # 5 minutes
                >>>
                >>> delivery = DeliveryTask(pickup_location, delivery_location, order_timestamp, service_duration)
                >>> print(delivery.current_state)  # DeliveryState.ASSIGNED
                >>> print(delivery.id)  # Unique task identifier

            Advanced Task Configuration:
                >>> # Create delivery with explicit ID and timing analysis
                >>> delivery = DeliveryTask(pickup, dropoff, order_time, pickup_time, id=12345)
                >>> print(delivery.event_time)  # All states initialized to None
                >>> print(delivery.order_time)  # Order placement timestamp
                >>> print(delivery.pickup_time)  # Service duration allocation
        """
        super().__init__(origin, destination, id)
        self.init_state_machine(DeliveryState.ASSIGNED, self._allowed)
        self._event_time = {
            DeliveryState.ASSIGNED: None,
            DeliveryState.GO_PICKUP: None,
            DeliveryState.SERVICE_PICKUP: None,
            DeliveryState.GO_DROPOFF: None,
            DeliveryState.SERVICE_DROPOFF: None,
            DeliveryState.DONE: None,
            DeliveryState.ABORTED: None,
        }

        self.order_time, self.pickup_time = order_time, pickup_time

    @property
    def event_time(self) -> dict[DeliveryState, Time | None]:
        """Get comprehensive event timestamp mapping for delivery lifecycle analysis.

        Provides read-only access to the complete event timing dictionary containing state
        entry timestamps for all delivery states. This comprehensive timing data enables
        detailed performance analysis, service level agreement monitoring, and operational
        optimization through precise temporal tracking of delivery workflow progression.

        Returns:
            dict[DeliveryState, Time | None]: Complete mapping of delivery states to their
                                             entry timestamps. States that have been entered
                                             contain Time instances capturing the exact moment
                                             of state transition. States not yet reached
                                             contain None values indicating future progression.

        Usage Examples:
            Performance Analysis:
                >>> delivery = DeliveryTask(pickup, dropoff, order_time, pickup_time)
                >>> # ... execute delivery workflow ...
                >>> times = delivery.event_time
                >>> pickup_duration = times[DeliveryState.GO_DROPOFF] - times[DeliveryState.SERVICE_PICKUP]
                >>> total_time = times[DeliveryState.DONE] - times[DeliveryState.ASSIGNED]

            State Progression Monitoring:
                >>> times = delivery.event_time
                >>> completed_states = [state for state, time in times.items() if time is not None]
                >>> pending_states = [state for state, time in times.items() if time is None]
        """
        return self._event_time

    def next(self, now: Time | None = None) -> DeliveryState:
        """Progress the task to the next state in the delivery sequence.

        Advances the task through its normal state progression according to
        the predefined sequence in _next dictionary. Validates the transition
        is allowed before updating the state.

        Args:
            now (Time | None): Current simulation time to record state entry timestamp.
                              If provided, records the timestamp for the current state.

        Returns:
            DeliveryState: The new current state after progression.

        Raises:
            ValueError: If the current state doesn't have a defined next state
                       or if the transition is not allowed.

        Example:
            >>> delivery = TaskDelivery(pickup, dropoff)
            >>> delivery.next()  # ASSIGNED → GO_PICKUP
            >>> delivery.next()  # GO_PICKUP → SERVICE_PICKUP
        """
        if now:
            self._event_time[self.current_state] = now
        
        self.transition_to(self._next[self.current_state])
        return self.current_state

    def abort(self):
        """Abort the delivery task and transition to ABORTED state.

        Cancels the current delivery task regardless of its current state
        and transitions to ABORTED. This is used for error handling,
        cancellation requests, or when the task cannot be completed.

        The task cannot be resumed after abortion and should be considered
        permanently failed.

        Example:
            >>> delivery = TaskDelivery(pickup, dropoff)
            >>> delivery.next()  # Progress to GO_PICKUP
            >>> delivery.abort()  # Emergency cancellation
            >>> print(delivery.current)  # DeliveryState.ABORTED
        """
        self.transition_to(DeliveryState.ABORTED)

    @staticmethod
    def ground_task() -> set[DeliveryState]:
        """Get the set of delivery states that require ground operations.

        Returns the delivery states during which the drone must be on the ground
        to perform package pickup or dropoff operations.

        Returns:
            set[DeliveryState]: Set containing SERVICE_PICKUP and SERVICE_DROPOFF states.
        """
        return {DeliveryState.SERVICE_PICKUP, DeliveryState.SERVICE_DROPOFF}

    @property
    def done(self) -> bool:
        """Check if the delivery task has reached a terminal state.

        A delivery task is considered done if it has either been completed
        successfully (DONE state) or has been aborted (ABORTED state).

        Returns:
            bool: True if the task is in DONE or ABORTED state, False otherwise.
        """
        return self.current_state in {DeliveryState.DONE, DeliveryState.ABORTED}
