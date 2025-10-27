"""Abstract task framework for mission management.

This module defines the core Task abstract base class that establishes
the fundamental interface for all mission tasks in the drone simulation
system. It provides state management, validation, and lifecycle control
for various mission types.

The Task class implements a finite state machine pattern where tasks
progress through defined states with validation rules to ensure proper
execution flow. Each task maintains geographic waypoints and provides
abstract methods for specific mission execution logic.

Key Components:
    Task: Abstract base class defining the task interface
    State: Generic type variable for task-specific state enumerations

Design Patterns:
    • Template Method: Base class provides structure, subclasses implement specifics
    • State Machine: Tasks progress through validated state transitions
    • Abstract Factory: Extensible framework for different task types

State Management:
    Tasks maintain current state and validate transitions using configurable
    rules defined in _allowed and _next dictionaries. This ensures tasks
    follow proper execution sequences and handle error conditions appropriately.
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import TypeVar

from app.geo import GeoPoint

State = TypeVar("State", bound=Enum)


class Task(ABC):
    """Abstract base class defining a mission task interface.

    This ABC establishes the contract that all mission task implementations
    must follow within the simulation environment. It provides essential
    infrastructure for task identification and simulation integration while
    requiring subclasses to implement their specific execution logic.

    The Task class follows the Template Method design pattern, providing
    concrete implementations for common operations (initialization, identification)
    while deferring task-specific behaviors to the abstract execute() method.

    Attributes:
        env (simpy.Environment): The discrete event simulation environment.
                                This provides access to simulation time, event
                                scheduling, and process management.
        id (int): Unique identifier generated from the object's memory address.
                 Used for tracking and distinguishing task instances.

    Abstract Methods:
        execute(): Task-specific execution implementation called when the
                   task is started within the simulation.

    Example:
        >>> class SimpleTask(Task):
        ...     def execute(self):
        ...         print(f"Task {self.id} executing at time {self.env.now}")
        ...
        >>> env = simpy.Environment()
        >>> task = SimpleTask(env)
    """

    id: int
    origin: GeoPoint
    destination: GeoPoint

    _allowed: dict[State, set[State]] = {}
    _next: dict[State, State] = {}

    def __init__(self, origin: GeoPoint, destination: GeoPoint):
        """Initialize a new Task instance with origin and destination points.

        Args:
            origin (GeoPoint): Starting geographic location for the task.
            destination (GeoPoint): Target geographic location for the task.
        """
        self.id = id(self)
        self.origin = origin
        self.destination = destination

    def request_transition(self, next_state: State):
        """Request a state transition to the specified next state.

        Validates the transition is legal according to the task's state machine
        rules before updating the current state. Raises ValueError if the
        transition is not allowed.

        Args:
            next_state (State): The target state to transition to.

        Raises:
            ValueError: If the transition from current state to next_state
                       is not allowed by the task's state machine rules.
        """
        self._validate_transition(self.current(), next_state)
        self.state = next_state

    @abstractmethod
    def abort(self):
        """Abort the current task and transition to aborted state.

        This abstract method must be implemented by subclasses to define
        task-specific abort behavior. Typically transitions the task to
        an ABORTED state and performs any necessary cleanup operations.
        """
        pass

    @property
    def current(self) -> State:
        """Get the current state of the task.

        Returns:
            State: The current state enumeration value for this task.
        """
        return self.state

    def _validate_transition(self, frm: State, to: State) -> None:
        """Validate that a state transition is allowed by the state machine rules.

        Checks the _allowed dictionary to determine if transitioning from
        the current state to the target state is permitted. This enforces
        the task's state machine constraints and prevents invalid state changes.

        Args:
            frm (State): The current state to transition from.
            to (State): The target state to transition to.

        Raises:
            ValueError: If the transition is not allowed according to the
                       task's state machine rules defined in _allowed.
        """
        if to not in self._allowed.get(frm, set()):
            msg = f"Illegal transition {frm.name} → {to.name}"
            raise ValueError(msg)
