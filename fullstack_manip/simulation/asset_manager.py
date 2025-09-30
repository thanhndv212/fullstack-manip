import subprocess
import sys
from etils import epath
import tqdm
from typing import Any, Dict, Union

# Directory layout helpers.
PACKAGE_ROOT = epath.Path(__file__).resolve().parent.parent
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
ASSETS_ROOT = PACKAGE_ROOT / "simulation" / "assets"
ROOT_PATH = ASSETS_ROOT
DEV_PATH = PACKAGE_ROOT.parent
MENAGERIE_PATH = DEV_PATH / "mujoco_menagerie"
# Commit SHA of the menagerie repo.
MENAGERIE_COMMIT_SHA = "14ceccf557cc47240202f2354d684eca58ff8de4"

print(f"Using mujoco_menagerie at {MENAGERIE_PATH}")


def _clone_with_progress(
    repo_url: str, target_path: str, commit_sha: str
) -> None:
    """Clone a git repo with progress bar."""
    process = subprocess.Popen(
        ["git", "clone", "--progress", repo_url, target_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )

    with tqdm.tqdm(
        desc="Cloning mujoco_menagerie",
        bar_format=(
            "{desc}: {bar}| {n_fmt}/{total_fmt} "
            "[{elapsed}<{remaining}]"
        ),
    ) as pbar:
        pbar.total = 100  # Set to 100 for percentage-based progress.
        current = 0
        while True:
            # Read output line by line.
            output = (
                process.stderr.readline()
            )  # pytype: disable=attribute-error
            if not output and process.poll() is not None:
                break
            if output:
                if "Receiving objects:" in output:
                    try:
                        percent = int(
                            output.split("%")[-2].split(":")[-1].strip()
                        )
                        if percent > current:
                            pbar.update(percent - current)
                            current = percent
                    except (ValueError, IndexError):
                        pass

        # Ensure the progress bar reaches 100%.
        if current < 100:
            pbar.update(100 - current)

    if process.returncode != 0:
        raise subprocess.CalledProcessError(
            process.returncode, ["git", "clone"]
        )

    # Checkout specific commit.
    print(f"Checking out commit {commit_sha}")
    subprocess.run(
        ["git", "-C", target_path, "checkout", commit_sha],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def ensure_menagerie_exists() -> None:
    """Ensure mujoco_menagerie exists, downloading it if necessary."""
    if not MENAGERIE_PATH.exists():
        print("mujoco_menagerie not found. Downloading...")

        # Create dev directory if it doesn't exist
        DEV_PATH.mkdir(exist_ok=True, parents=True)

        try:
            _clone_with_progress(
                "https://github.com/deepmind/mujoco_menagerie.git",
                str(MENAGERIE_PATH),
                MENAGERIE_COMMIT_SHA,
            )
            print("Successfully downloaded mujoco_menagerie")
        except subprocess.CalledProcessError as e:
            print(f"Error downloading mujoco_menagerie: {e}", file=sys.stderr)
            raise


def update_assets(
    assets: Dict[str, Any],
    path: Union[str, epath.Path],
    glob: str = "*",
    recursive: bool = False,
):
    """Update asset dictionary using files matching the glob pattern."""
    for f in epath.Path(path).glob(glob):
        if f.is_file():
            assets[f.name] = f.read_bytes()
        elif f.is_dir() and recursive:
            update_assets(assets, f, glob, recursive)


def get_assets_from_menagerie(
    robot_dir: str | None,
    gripper_dir: str | None,
    env_dir: Union[str, epath.Path, None],
) -> Dict[str, bytes]:
    """Get assets from the mujoco menagerie."""

    assets: Dict[str, bytes] = {}
    ensure_menagerie_exists()

    if robot_dir:
        robot_path = MENAGERIE_PATH / robot_dir
        update_assets(assets, robot_path, "*.xml")
        update_assets(assets, robot_path / "assets")

    if gripper_dir:
        gripper_path = MENAGERIE_PATH / gripper_dir
        update_assets(assets, gripper_path, "*.xml")
        update_assets(assets, gripper_path / "assets")

    if env_dir:
        env_path = epath.Path(env_dir)
        update_assets(assets, env_path / "xmls", "*.xml")
        update_assets(assets, env_path / "assets", "*")

    return assets


__all__ = [
    "SCRIPTS_ROOT",
    "ASSETS_ROOT",
    "ROOT_PATH",
    "DEV_PATH",
    "MENAGERIE_PATH",
    "ensure_menagerie_exists",
    "get_assets_from_menagerie",
    "update_assets",
]
