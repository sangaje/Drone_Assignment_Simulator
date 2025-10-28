"""State management module for drone simulation.

This module provides state machine functionality for managing drone states,
task states, and other stateful components in the drone simulation system.

Exports:
    StateMachine: Finite state machine with transition validation
    State: Type variable for state enumerations
    Action: State transition action with optional effects
    StateGraph: Type alias for state transition graph definitions
"""

from .state_machine import Action, ActionFn, State, StateGraph, StateMachine

__all__ = ["StateMachine", "State", "Action", "StateGraph", "ActionFn"]
