"""
Blackboard: Shared state storage for behavior trees.

The blackboard provides a centralized key-value store that behaviors
can use to share data and coordinate actions.
"""

from typing import Any, Dict, Optional, Set
import threading


class Blackboard:
    """
    Thread-safe shared state storage for behavior trees.

    The blackboard allows behaviors to read and write shared state
    without tight coupling between behaviors.
    """

    def __init__(self) -> None:
        """Initialize empty blackboard with thread safety."""
        self._data: Dict[str, Any] = {}
        self._lock = threading.RLock()

    def set(self, key: str, value: Any) -> None:
        """
        Set a value on the blackboard.

        Args:
            key: The key to store the value under
            value: The value to store
        """
        with self._lock:
            self._data[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get a value from the blackboard.

        Args:
            key: The key to retrieve
            default: Default value if key doesn't exist

        Returns:
            The stored value or default if key not found
        """
        with self._lock:
            return self._data.get(key, default)

    def exists(self, key: str) -> bool:
        """
        Check if a key exists on the blackboard.

        Args:
            key: The key to check

        Returns:
            True if key exists, False otherwise
        """
        with self._lock:
            return key in self._data

    def delete(self, key: str) -> bool:
        """
        Delete a key from the blackboard.

        Args:
            key: The key to delete

        Returns:
            True if key was deleted, False if it didn't exist
        """
        with self._lock:
            if key in self._data:
                del self._data[key]
                return True
            return False

    def keys(self) -> Set[str]:
        """
        Get all keys currently on the blackboard.

        Returns:
            Set of all keys
        """
        with self._lock:
            return set(self._data.keys())

    def clear(self) -> None:
        """Clear all data from the blackboard."""
        with self._lock:
            self._data.clear()

    def __repr__(self) -> str:
        with self._lock:
            return f"Blackboard({len(self._data)} keys)"
