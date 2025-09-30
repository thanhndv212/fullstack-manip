import time
from typing import Optional, Callable

import mujoco
import mujoco.viewer


class MuJoCoViewer:
    """Wrapper around the MuJoCo viewer for interactive visualization."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        show_left_ui: bool = False,
        show_right_ui: bool = False,
        key_callback: Optional[Callable] = None,
    ):
        self.model = model
        self.data = data
        self.show_left_ui = show_left_ui
        self.show_right_ui = show_right_ui
        self.key_callback = key_callback
        self.viewer: Optional[mujoco.viewer.Viewer] = None
        self.timestamp = 0.0
        self.dt = 0.01

    def launch_passive(self) -> mujoco.viewer.Viewer:
        """Launch a passive MuJoCo viewer instance."""
        if self.model is None or self.data is None:
            raise ValueError(
                "Model and data must be provided to launch the viewer"
            )

        self.viewer = mujoco.viewer.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=self.show_left_ui,
            show_right_ui=self.show_right_ui,
            key_callback=self.key_callback,
        )
        self.viewer.opt.geomgroup[0] = 1
        self.viewer.opt.geomgroup[1] = 1
        self.viewer.opt.geomgroup[2] = 1
        self.viewer.opt.geomgroup[3] = 1
        time.sleep(0.1)  # Allow the viewer to initialize
        return self.viewer

    def step(self, current_joint_positions) -> None:
        """Advance the simulation and refresh the viewer."""
        if self.viewer:
            self.data.ctrl[:] = current_joint_positions
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            self.timestamp += self.dt

    def sync(self) -> None:
        """Force a viewer refresh without stepping the simulation."""
        if self.viewer:
            self.viewer.sync()

    def is_running(self) -> bool:
        """Return True while the viewer remains open."""
        return self.viewer is not None and self.viewer.is_running()

    def close(self) -> None:
        """Close the viewer window."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None

    def start_recording(self, filename: str) -> None:
        """Begin recording frames to the specified file."""
        if self.viewer is None:
            raise RuntimeError("Viewer not launched")
        self.viewer.record_video = filename

    def stop_recording(self) -> None:
        """Stop recording frames if active."""
        if self.viewer:
            self.viewer.record_video = None


__all__ = ["MuJoCoViewer"]
