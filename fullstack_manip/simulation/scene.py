import mujoco
from typing import Optional, Union, Dict, Any
from pathlib import Path

from .loader import MuJoCoSceneLoader


class MuJoCoSceneManager:
    """Manage MuJoCo scenes by handling model and data lifecycles."""

    def __init__(
        self,
        xml_path: Union[str, Path],
        loader: Optional[MuJoCoSceneLoader] = None,
        load_method: str = "xml_path",
    ):
        self.xml_path: Optional[Path] = Path(xml_path).resolve()
        self.loader = loader if loader else MuJoCoSceneLoader()
        self.load_method = load_method
        self.model: Optional[mujoco.MjModel] = self.loader.load_scene(
            method=load_method,
            xml_path=self.xml_path,
        )
        if self.model is None:
            raise ValueError(
                "Failed to load model from "
                f"{self.xml_path} using method {load_method}"
            )
        self.data: Optional[mujoco.MjData] = mujoco.MjData(self.model)
        self.assets: Optional[Dict[str, bytes]] = None
        mujoco.mj_resetDataKeyframe(
            self.model,
            self.data,
            self.model.key("home").id,
        )
        self.data.qpos[:] = self.model.key("home").qpos
        mujoco.mj_forward(self.model, self.data)

    def reload(self) -> None:
        """Reload the current scene."""
        if self.xml_path:
            self.loader.load_scene(
                method=self.load_method,
                xml_path=self.xml_path,
            )
        elif self.model:
            self.data = mujoco.MjData(self.model)
        else:
            raise ValueError("No scene loaded to reload")

    def reset(self) -> None:
        """Reset the simulation data to initial state."""
        if self.data:
            mujoco.mj_resetData(self.model, self.data)

    def step(self) -> None:
        """Advance the simulation by one step."""
        if self.model and self.data:
            mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        """Compute forward dynamics without stepping."""
        if self.model and self.data:
            mujoco.mj_forward(self.model, self.data)

    def get_body_position(self, body_name: str) -> Optional[Any]:
        """Get position of a body by name."""
        if self.model and self.data:
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            if body_id >= 0:
                return self.data.xpos[body_id]
        return None

    def get_body_orientation(self, body_name: str) -> Optional[Any]:
        """Get orientation of a body by name."""
        if self.model and self.data:
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
            )
            if body_id >= 0:
                return self.data.xmat[body_id].reshape(3, 3)
        return None

    def set_joint_positions(
        self, joint_positions: Any, joint_names: Optional[list] = None
    ) -> None:
        """Set joint positions."""
        if self.data:
            if joint_names:
                for name, pos in zip(joint_names, joint_positions):
                    joint_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                    )
                    if joint_id >= 0:
                        self.data.qpos[joint_id] = pos
            else:
                self.data.qpos[:] = joint_positions
            mujoco.mj_forward(self.model, self.data)

    def get_joint_positions(
        self, joint_names: Optional[list] = None
    ) -> Optional[Any]:
        """Get joint positions."""
        if self.data:
            if joint_names:
                positions = []
                for name in joint_names:
                    joint_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                    )
                    if joint_id >= 0:
                        positions.append(self.data.qpos[joint_id])
                return positions
            return self.data.qpos
        return None

    @property
    def is_loaded(self) -> bool:
        """Check if a scene is currently loaded."""
        return self.model is not None and self.data is not None

    def __str__(self) -> str:
        status = "loaded" if self.is_loaded else "not loaded"
        return f"MuJoCoSceneManager(status={status}, xml_path={self.xml_path})"


__all__ = ["MuJoCoSceneManager"]
