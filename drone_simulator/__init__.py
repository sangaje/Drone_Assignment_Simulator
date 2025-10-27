"""
Drone Assignment Simulator
A tool to test and analyze drone assignment problems with multiple strategies.
"""

from .models import Drone, Task, Location, AssignmentResult
from .problem import AssignmentProblem
from .strategies import (
    AssignmentStrategy,
    RandomAssignment,
    NearestAssignment,
    MILPAssignment,
    UserDefinedAssignment
)
from .analyzer import AssignmentAnalyzer

__version__ = "0.1.0"

__all__ = [
    "Drone",
    "Task",
    "Location",
    "AssignmentResult",
    "AssignmentProblem",
    "AssignmentStrategy",
    "RandomAssignment",
    "NearestAssignment",
    "MILPAssignment",
    "UserDefinedAssignment",
    "AssignmentAnalyzer",
]
