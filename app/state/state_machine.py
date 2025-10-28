"""State machine implementation for managing validated state transitions.

This module provides a finite state machine implementation that enforces transition rules and can
execute associated actions when transitions occur. Used for controlling drone behavior and task
execution flow.
"""

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any, TypeVar

State = TypeVar("State", bound=Enum)
"""Type variable for state enumerations that extend Enum."""

ActionFn = Callable[[], None]
"""Type alias for action effect functions that take no parameters."""

StateGraph = TypeVar("StateGraph", bound=dict[State, set["Action"]])


@dataclass(frozen=True)
class Action:
    """Represents a state transition action with an optional effect function.

    An Action defines a transition to a specific state and can optionally
    execute a side effect when the transition occurs.

    Attributes:
        state: The target state this action transitions to.
        effect: Optional function to execute when this action is performed.
    """

    state: State
    effect: ActionFn | None = None

    def __call__(self, *args, **kwargs) -> Any:
        """Execute the action's effect function if it exists.

        Args:
            *args: Arguments to pass to the effect function.
            **kwargs: Keyword arguments to pass to the effect function.

        Returns:
            The result of the effect function, or None if no effect is defined.
        """
        if self.effect:
            return self.effect(*args, **kwargs)


class StateMachine:
    """A finite state machine that manages state transitions with validation.

    This class implements a state machine that validates transitions between states
    according to predefined rules. Each transition can have an associated action
    that is executed when the transition occurs.

    Attributes:
        _state: The current state of the state machine.
        _allowed: Dictionary mapping states to their allowed transitions.
    """

    _allowed: StateGraph
    _state: State

    def __init__(self, initial_state: State, nodes_graph: StateGraph):
        """Initialize the state machine with an initial state and transition rules.

        Args:
            initial_state: The starting state for the state machine.
            nodes_graph: Dictionary mapping each state to its set of allowed actions/transitions.
        """
        self._state = initial_state
        self._allowed = nodes_graph

    def request_transition(self, next_state: State, *args, **kwargs) -> Any:
        """Request a state transition to the specified next state.

        Validates that the transition is allowed according to the state machine
        rules, executes any associated action effect, and updates the current state.

        Args:
            next_state: The target state to transition to.

        Returns:
            The result of executing the transition action's effect function,
            or None if the action has no effect.

        Raises:
            ValueError: If the transition from current state to next_state
                       is not allowed by the state machine rules.
        """
        next_action = self._validate_transition(self.current, next_state)
        self.state = next_action.state
        return next_action(*args, **kwargs)

    @property
    def current(self) -> State:
        """Get the current state of the state machine.

        Returns:
            The current state enumeration value.
        """
        return self._state

    def _validate_transition(self, frm: State, to: State) -> Action:
        """Validate that a state transition is allowed by the state machine rules.

        Searches through the allowed actions for the source state to find
        an action that transitions to the target state.

        Args:
            frm: The current state to transition from.
            to: The target state to transition to.

        Returns:
            The Action object that handles the transition from frm to to.

        Raises:
            ValueError: If no valid transition exists from frm to to
                       according to the state machine rules.
        """
        allowed_actions: set[Action] = self._allowed.get(frm, set())
        for action in allowed_actions:
            if action.state == to:
                return action

        msg = f"Illegal transition {frm.name} → {to.name}"
        raise ValueError(msg)
