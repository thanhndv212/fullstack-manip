import mujoco
import mujoco.viewer
from typing import Optional, Callable
from scene import MuJoCoSceneManager


class MuJoCoViewer:
    """A wrapper class for MuJoCo viewer to handle visualization of scenes."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        show_left_ui: bool = True,
        show_right_ui: bool = False,
        key_callback: Optional[Callable] = None,
    ):
        """
        Initialize the viewer with a scene manager.

        Args:
            scene_manager: The MuJoCoSceneManager instance.
            show_left_ui: Whether to show the left UI panel.
            show_right_ui: Whether to show the right UI panel.
            key_callback: Optional key callback function.
        """
        self.model = model
        self.data = data
        self.show_left_ui = show_left_ui
        self.show_right_ui = show_right_ui
        self.key_callback = key_callback
        self.viewer: Optional[mujoco.viewer.Viewer] = None
        self.timestamp = 0.0
        self.dt = 0.01  # Default time step

    def launch_passive(self):
        """
        Launch the passive viewer.

        Returns:
            The launched viewer instance.
        """
        if self.model is None or self.data is None:
            raise ValueError("Scene not loaded in scene manager")

        self.viewer = mujoco.viewer.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=self.show_left_ui,
            show_right_ui=self.show_right_ui,
            key_callback=self.key_callback,
        )
        return self.viewer

    def step(self, current_joint_positions) -> None:
        """Advance the viewer by one step."""
        if self.viewer:
            self.data.ctrl[:] = current_joint_positions
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            self.timestamp += self.dt

    def sync(self) -> None:
        """Sync the viewer with the current simulation state."""
        if self.viewer:
            self.viewer.sync()

    def is_running(self) -> bool:
        """Check if the viewer is still running."""
        return self.viewer is not None and self.viewer.is_running()

    def close(self) -> None:
        """Close the viewer."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None
