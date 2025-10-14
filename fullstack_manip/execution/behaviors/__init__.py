"""
Base behavior class and status enum for behavior tree implementation.
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional


class Status(Enum):
    """Behavior tree node execution status."""

    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"
    INVALID = "INVALID"


class BaseBehavior(ABC):
    """
    Abstract base class for all behavior tree nodes.

    All atomic behaviors (leaf nodes) should inherit from this class
    and implement the update() method.
    """

    def __init__(
        self, name: str, blackboard: Optional["Blackboard"] = None
    ) -> None:
        """
        Initialize behavior.

        Args:
            name: Human-readable name for this behavior
            blackboard: Shared state storage (optional)
        """
        self.name = name
        self.blackboard = blackboard
        self.status = Status.INVALID
        self.feedback_message = ""

    @abstractmethod
    def update(self) -> Status:
        """
        Execute the behavior logic.

        This method should be implemented by all concrete behaviors
        to define their execution logic.

        Returns:
            Status indicating the result of the behavior execution
        """
        pass

    def setup(self) -> None:
        """
        Setup behavior before first execution.

        Override this method to perform initialization tasks.
        """
        pass

    def terminate(self, new_status: Status) -> None:
        """
        Cleanup when behavior terminates.

        Override this method to perform cleanup tasks.

        Args:
            new_status: The status this behavior is terminating with
        """
        pass

    def reset(self) -> None:
        """Reset behavior to initial state."""
        self.status = Status.INVALID
        self.feedback_message = ""

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}('{self.name}')"


# Export base classes
__all__ = [
    "Status",
    "BaseBehavior",
]
