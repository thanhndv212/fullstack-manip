import mujoco
from typing import Optional, Dict, Union
from pathlib import Path

from .asset_manager import ROOT_PATH, get_assets_from_menagerie

try:
    from etils import epath
except ImportError:  # pragma: no cover - optional dependency
    epath = None


class MuJoCoSceneLoader:
    """Load MuJoCo scenes from various sources."""

    def __init__(
        self,
        robot_dir: Optional[str] = None,
        gripper_dir: Optional[str] = None,
        env_dir: Optional[Union[str, Path]] = None,
    ):
        """Initialize the loader with optional directory paths for assets."""
        self.robot_dir = robot_dir
        self.gripper_dir = gripper_dir
        if env_dir:
            self.env_dir = Path(env_dir)
        else:
            self.env_dir = (
                ROOT_PATH / "trs_so_arm100" if ROOT_PATH is not None else None
            )

    def load_from_robot_descriptions(self, robot_name: str) -> mujoco.MjModel:
        """Load model from robot_descriptions package."""
        from robot_descriptions.loaders.mujoco import load_robot_description

        try:
            model = load_robot_description(robot_name)
        except ModuleNotFoundError:
            model = load_robot_description(f"{robot_name}_mj_description")
        return model

    def load_from_xml_string(
        self,
        xml_path: Union[str, Path],
        assets: Optional[Dict[str, bytes]] = None,
    ) -> mujoco.MjModel:
        """Load model from XML string with optional assets."""
        if assets is None and self._can_load_assets():
            assets = get_assets_from_menagerie(
                self.robot_dir, self.gripper_dir, self.env_dir
            )

        xml_path = Path(xml_path)
        xml_string = xml_path.read_text()
        model = mujoco.MjModel.from_xml_string(xml_string, assets=assets)
        return model

    def load_from_xml_path(self, xml_path: Union[str, Path]) -> mujoco.MjModel:
        """Load model directly from XML file path."""
        xml_path = Path(xml_path).resolve()
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        return model

    def load_scene(self, method: str, **kwargs) -> mujoco.MjModel:
        """Dispatch to the requested load method."""
        if method == "robot_descriptions":
            robot_name = kwargs.get("robot_name")
            if not robot_name:
                raise ValueError(
                    "robot_name required for robot_descriptions method"
                )
            return self.load_from_robot_descriptions(robot_name)
        if method == "xml_string":
            xml_path = kwargs.get("xml_path")
            if not xml_path:
                raise ValueError("xml_path required for xml_string method")
            return self.load_from_xml_string(xml_path, kwargs.get("assets"))
        if method == "xml_path":
            xml_path = kwargs.get("xml_path")
            if not xml_path:
                raise ValueError("xml_path required for xml_path method")
            return self.load_from_xml_path(xml_path)
        raise ValueError(f"Unknown loading method: {method}")

    def _can_load_assets(self) -> bool:
        """Check if all required directories are set for asset loading."""
        return all(
            value is not None
            for value in (self.robot_dir, self.gripper_dir, self.env_dir)
        )


__all__ = ["MuJoCoSceneLoader", "ROOT_PATH"]
