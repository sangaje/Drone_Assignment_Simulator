"""Delivery task implementation for drone package delivery missions.

This module implements a concrete delivery task that extends the abstract Task
base class. It provides a complete state machine for package delivery operations
including pickup, transit, and dropoff phases with appropriate error handling.

The delivery task follows a linear progression through defined states with
validation rules to ensure proper execution flow. Each state represents a
specific phase of the delivery operation from initial assignment to completion.

State Flow:
    ASSIGNED → GO_PICKUP → SERVICE_PICKUP → GO_DROPOFF → SERVICE_DROPOFF → DONE

    Any state can transition to ABORTED for error handling and task cancellation.

Components:
    DeliveryState: Enumeration of all possible delivery task states
    TaskDelivery: Concrete delivery task implementation with state management

Configuration:
    _allowed: Dictionary mapping allowed state transitions
    _next: Dictionary defining the normal progression sequence
"""

from __future__ import annotations

from enum import IntEnum, auto

from dronesim.geo import GeoPoint
from dronesim.state import Action, StateMachine
from dronesim.unit import Time

from .task import Task


class DeliveryState(IntEnum):
    """Enumeration of states for delivery task progression.

    This enum defines all possible states that a delivery task can be in
    during its lifecycle. States represent discrete phases of the delivery
    operation from initial assignment through completion or abortion.

    States:
        ASSIGNED: Task has been assigned but not yet started
        GO_PICKUP: Drone is traveling to pickup location
        SERVICE_PICKUP: Drone is at pickup location loading package
        GO_DROPOFF: Drone is traveling to dropoff location with package
        SERVICE_DROPOFF: Drone is at dropoff location unloading package
        DONE: Delivery has been completed successfully
        ABORTED: Task has been cancelled or failed
    """

    ASSIGNED = auto()
    GO_PICKUP = auto()
    SERVICE_PICKUP = auto()
    GO_DROPOFF = auto()
    SERVICE_DROPOFF = auto()
    DONE = auto()
    ABORTED = auto()


class DeliveryTask(Task):
    """Concrete implementation of a package delivery task.

    This class implements the abstract Task interface to provide complete
    package delivery functionality with state management, validation, and
    progression control. The task manages the full delivery lifecycle from
    assignment through completion.

    The delivery task follows a predefined state sequence with validation
    rules to ensure proper execution. It handles both normal progression
    and error conditions through the abort mechanism.

    Attributes:
        _event_time (dict[DeliveryState, Time | None]): Timestamps for each state entry.
        state_machine (StateMachine | None): State machine instance for transitions.

    Inherited Attributes:
        origin (GeoPoint): Pickup location for the package.
        destination (GeoPoint): Delivery destination for the package.
        id (int): Unique identifier for this task instance.

    State Progression:
        Normal flow: ASSIGNED → GO_PICKUP → SERVICE_PICKUP →
                    GO_DROPOFF → SERVICE_DROPOFF → DONE
        Error flow: Any state → ABORTED

    Example:
        >>> pickup = GeoPoint.from_deg(37.5665, 126.9780)
        >>> dropoff = GeoPoint.from_deg(37.5675, 126.9890)
        >>> delivery = TaskDelivery(pickup, dropoff)
        >>> print(delivery.current)  # DeliveryState.ASSIGNED
        >>> delivery.next()  # Progress to GO_PICKUP
        >>> print(delivery.current)  # DeliveryState.GO_PICKUP
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

    state_machine: StateMachine | None

    def __init__(self, origin: GeoPoint, destination: GeoPoint):
        """Initialize a new delivery task with pickup and dropoff locations.

        Creates a new delivery task in the ASSIGNED state with the specified
        pickup and delivery locations. The task is ready to begin execution
        through the state progression sequence.

        Args:
            origin (GeoPoint): Geographic location for package pickup.
            destination (GeoPoint): Geographic location for package delivery.
        """
        super().__init__(origin, destination)
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

    @property
    def event_time(self) -> dict[DeliveryState, Time | None]:
        """Get the event timestamps for each delivery state.

        Returns:
            dict[DeliveryState, Time | None]: Mapping of each delivery state
            to the Time it was entered, or None if not yet reached.
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
