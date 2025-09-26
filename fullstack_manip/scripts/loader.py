import mujoco
from typing import Optional, Dict, Union
from pathlib import Path

try:
    from etils import epath
except ImportError:
    epath = None


class MuJoCoSceneLoader:
    """A class to load MuJoCo scenes from various sources.

    Supports loading from robot_descriptions package, XML strings with assets,
    and direct XML file paths.
    """

    def __init__(
        self,
        robot_dir: Optional[str] = None,
        gripper_dir: Optional[str] = None,
        env_dir: Optional[Union[str, Path]] = None,
    ):
        """Initialize the loader with optional directory paths for assets.

        Args:
            robot_dir: Directory name for robot assets in mujoco_menagerie.
            gripper_dir: Directory name for gripper assets.
            env_dir: Base directory for environment XMLs and assets.
        """
        self.robot_dir = robot_dir
        self.gripper_dir = gripper_dir
        self.env_dir = Path(env_dir) if env_dir else None

    def load_from_robot_descriptions(self, robot_name: str) -> mujoco.MjModel:
        """Load model from robot_descriptions package.

        Args:
            robot_name: Name of the robot (e.g., 'ur5e').

        Returns:
            Loaded MuJoCo model.

        Raises:
            ModuleNotFoundError: If robot_descriptions is not available.
        """
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
        """Load model from XML string with optional assets.

        Args:
            xml_path: Path to the XML file.
            assets: Optional dictionary of asset names to bytes.

        Returns:
            Loaded MuJoCo model.
        """
        if assets is None and self._can_load_assets():
            from assets import get_assets_from_menagerie

            assets = get_assets_from_menagerie(
                self.robot_dir, self.gripper_dir, self.env_dir
            )

        xml_path = Path(xml_path)
        xml_string = xml_path.read_text()
        model = mujoco.MjModel.from_xml_string(xml_string, assets=assets)
        return model

    def load_from_xml_path(self, xml_path: Union[str, Path]) -> mujoco.MjModel:
        """Load model directly from XML file path.

        Args:
            xml_path: Path to the XML file.

        Returns:
            Loaded MuJoCo model.
        """
        xml_path = Path(xml_path).resolve()  # Ensure absolute path
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        return model

    def load_scene(self, method: str, **kwargs) -> mujoco.MjModel:
        """General load method that dispatches to specific loaders.

        Args:
            method: Loading method ('robot_descriptions', 'xml_string', 'xml_path').
            **kwargs: Keyword arguments for the specific method.

        Returns:
            Loaded MuJoCo model.

        Raises:
            ValueError: If method is unknown.
        """
        if method == "robot_descriptions":
            robot_name = kwargs.get("robot_name")
            if not robot_name:
                raise ValueError(
                    "robot_name required for robot_descriptions method"
                )
            return self.load_from_robot_descriptions(robot_name)
        elif method == "xml_string":
            xml_path = kwargs.get("xml_path")
            if not xml_path:
                raise ValueError("xml_path required for xml_string method")
            return self.load_from_xml_string(xml_path, kwargs.get("assets"))
        elif method == "xml_path":
            xml_path = kwargs.get("xml_path")
            if not xml_path:
                raise ValueError("xml_path required for xml_path method")
            return self.load_from_xml_path(xml_path)
        else:
            raise ValueError(f"Unknown loading method: {method}")

    def _can_load_assets(self) -> bool:
        """Check if all required directories are set for asset loading."""
        return (
            self.robot_dir is not None
            and self.gripper_dir is not None
            and self.env_dir is not None
        )
