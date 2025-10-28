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
    StateGraph: Type alias for state transition graph definitions

Design Patterns:
    • Template Method: Base class provides structure, subclasses implement specifics
    • State Machine: Tasks progress through validated state transitions
    • Abstract Factory: Extensible framework for different task types

State Management:
    Tasks use a StateMachine instance to manage state transitions with validation.
    Subclasses MUST initialize the `state_machine` ClassVar with a StateMachine
    instance configured with their specific state graph and initial state.

Implementation Requirements:
    Concrete task classes must define:
    1. A state enumeration (e.g., TaskState(Enum))
    2. A state transition graph (StateGraph)
    3. Initialize state_machine ClassVar in class definition
    4. Implement abstract methods (execute, abort)

Example:
    class MyTaskState(Enum):
        PENDING = "pending"
        RUNNING = "running"
        COMPLETED = "completed"

    class MyTask(Task):
        # REQUIRED: Initialize state_machine ClassVar
        state_machine: ClassVar[StateMachine] = StateMachine(
            initial_state=MyTaskState.PENDING,
            nodes_graph={...}  # Define your state transitions
        )
"""

from abc import ABC, abstractmethod
from typing import Any

from dronesim.geo import GeoPoint
from dronesim.state import State, StateGraph, StateMachine


class Task(ABC):
    """Abstract base class defining a mission task interface with state management.

    This ABC establishes the contract that all mission task implementations
    must follow within the simulation environment. It provides essential
    infrastructure for task identification, geographic waypoints, and state
    machine management while requiring subclasses to implement their specific
    execution logic.

    The Task class follows the Template Method design pattern, providing
    concrete implementations for common operations (initialization, identification)
    while deferring task-specific behaviors to abstract methods.

    Attributes:
        id (int): Unique identifier generated from the object's memory address.
        origin (GeoPoint): Starting geographic location for the task.
        destination (GeoPoint): Target geographic location for the task.
        _state_machine (StateMachine | None): Internal state machine instance for managing
                                            task state transitions and validation.

    Abstract Methods:
        abort(): Task-specific abort implementation for cleanup and state transition.

    Class Variable Requirements:
        Subclasses MUST define and initialize the state_machine ClassVar:

        state_machine: ClassVar[StateMachine] = StateMachine(
            initial_state=YourTaskState.INITIAL,
            nodes_graph={dronesim..}  # Your state transition rules
        )

    Example:
        >>> class DeliveryTaskState(Enum):
        ...     PENDING = "pending"
        ...     IN_PROGRESS = "in_progress"
        ...     COMPLETED = "completed"
        >>> class DeliveryTask(Task):
        ...     state_machine: ClassVar[StateMachine] = StateMachine(
        ...         initial_state=DeliveryTaskState.PENDING, nodes_graph={...}
        ...     )
        ...
        ...     def abort(self):
        ...         self.state_machine.request_transition(DeliveryTaskState.ABORTED)
    """

    id: int
    origin: GeoPoint
    destination: GeoPoint

    _state_machine: StateMachine | None = None

    def __init__(self, origin: GeoPoint, destination: GeoPoint):
        """Initialize a new Task instance with origin and destination points.

        Args:
            origin (GeoPoint): Starting geographic location for the task.
            destination (GeoPoint): Target geographic location for the task.
        """
        self.id = id(self)
        self.origin = origin
        self.destination = destination

    def init_state_machine(self, initial_state: State, nodes_graph: StateGraph) -> None:
        """Initialize the state machine for the task.

        This method can be called to set up the state machine if not
        initialized at class definition time. Subclasses may override
        this method to provide custom initialization logic.

        Raises:
            NotImplementedError: If the subclass does not implement
                                 state machine initialization.
        """
        self._state_machine = StateMachine(initial_state, nodes_graph)

    @abstractmethod
    def abort(self):
        """Abort the current task and transition to aborted state.

        This abstract method must be implemented by subclasses to define task-specific abort
        behavior. Typically transitions the task to an ABORTED state and performs any necessary
        cleanup operations.
        """
        pass

    def transition_to(self, next_state: State, *args, **kwargs) -> Any:
        """Request a state transition to the specified state.

        Attempts to transition the task's state machine to the target state,
        validating that the transition is allowed according to the configured
        state graph. Executes any associated action effects during the transition.

        Args:
            next_state (State): The target state to transition to.
            *args: Additional arguments passed to transition action effects.
            **kwargs: Additional keyword arguments passed to action effects.

        Returns:
            Any: The result of executing the transition action's effect function,
                or None if the action has no effect.

        Raises:
            NotImplementedError: If the state machine has not been initialized.
            ValueError: If the transition is not allowed by the state machine rules.
        """
        if not type(self)._state_machine:
            msg = "Subclasses must initialize the state_machine ClassVar."
            raise NotImplementedError(msg)

        return type(self)._state_machine.request_transition(next_state, *args, **kwargs)

    @property
    def current_state(self) -> State:
        """Get the current state of the task from the state machine.

        Returns:
            State: The current state of the task as managed by the state machine.
        """
        if not self._state_machine:
            msg = "Subclasses must initialize the state_machine"
            raise NotImplementedError(msg)

        return self._state_machine.current
