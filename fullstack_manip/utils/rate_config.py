"""Central configuration for timing rates across the manipulation stack.

This module provides a small, dependency-light helper for defining and
propagating the various loop time-steps (``dt``) used throughout the
system.  Components can obtain a shared :class:`RateConfig` instance to
avoid duplicating constants or hard-coding mismatched values.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, MutableMapping
import json
import os

try:  # Optional dependency; only used when available.
    import yaml  # type: ignore
except Exception:  # pragma: no cover - defensive
    yaml = None  # type: ignore


_Number = float | int


@dataclass(frozen=True, slots=True)
class RateConfig:
    """Container for the primary loop time-steps in seconds.

    Attributes
    ----------
    sim:
        Simulation integrator step (``dt``) used by physics integration.
    control:
        High-level control loop step used by Cartesian/force controllers.
    planner:
        Trajectory re-planning cadence.  Slower by default because planning
        can be expensive.
    estimator:
        State-estimation update period.
    sensor:
        Sensor polling period (depth/RGB/force sensors, etc.).
    extras:
        Arbitrary additional named ``dt`` values.  Useful for specialised
        loops such as teleoperation inputs or perception pipelines.
    """

    sim: float = 0.002
    control: float = 0.01
    planner: float = 0.1
    estimator: float = 0.01
    sensor: float = 0.02
    extras: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate the supplied ``dt`` values."""
        for name in ("sim", "control", "planner", "estimator", "sensor"):
            value = getattr(self, name)
            _ensure_positive_dt(name, value)
        # Freeze extras as a regular dictionary copy to guard against external
        # mutation when the input mapping is mutable (e.g., ``dict``).
        object.__setattr__(self, "extras", dict(self.extras))
        for key, value in self.extras.items():
            _ensure_positive_dt(key, value)

    def get(self, key: str, default: float | None = None) -> float | None:
        """Return a ``dt`` by name, consulting ``extras`` when needed."""
        if hasattr(self, key):
            return getattr(self, key)
        return self.extras.get(key, default)

    def frequency(self, key: str) -> float:
        """Return the frequency (Hz) associated with ``key``.

        Raises
        ------
        KeyError
            If the ``key`` is unknown.
        ValueError
            If the resolved ``dt`` is non-positive.
        """
        dt = self.get(key)
        if dt is None:
            raise KeyError(f"Unknown rate key '{key}'.")
        if dt <= 0.0:
            raise ValueError(f"Rate '{key}' must be positive, got {dt}.")
        return 1.0 / dt

    def with_overrides(self, **overrides: float) -> "RateConfig":
        """Return a new configuration with selected ``dt`` values replaced."""
        extras = dict(self.extras)
        updates: MutableMapping[str, float] = {}
        for key, value in overrides.items():
            updates[key] = float(value)

        known_keys = {"sim", "control", "planner", "estimator", "sensor"}
        init_kwargs = {k: getattr(self, k) for k in known_keys}

        for key, value in updates.items():
            if key in known_keys:
                init_kwargs[key] = value
            else:
                extras[key] = value

        init_kwargs["extras"] = extras
        return RateConfig(**init_kwargs)

    def to_dict(self) -> dict[str, float]:
        """Serialise the configuration into a flat dictionary."""
        data = {
            "sim": self.sim,
            "control": self.control,
            "planner": self.planner,
            "estimator": self.estimator,
            "sensor": self.sensor,
        }
        data.update(self.extras)
        return data


def _ensure_positive_dt(name: str, value: float) -> None:
    if value <= 0.0:
        raise ValueError(f"Rate '{name}' must be positive, got {value}.")


def load_rate_config(
    path: str | Path | None = None,
    *,
    env_prefix: str = "RATES__",
    defaults: RateConfig | None = None,
) -> RateConfig:
    """Load a :class:`RateConfig` from file and environment overrides.

    Parameters
    ----------
    path:
        Optional explicit path pointing to a JSON or YAML document containing
        ``dt`` values.  When omitted, the loader searches
        ``configs/rates.yaml`` then ``configs/rates.json`` relative to the
        current working directory.
    env_prefix:
        Environment variable prefix used to override entries.  For example,
        ``RATES__CONTROL=0.005`` overrides the control loop step.  Values may
    be expressed either in seconds (``0.01``) or as frequencies
    (``200Hz``).
    defaults:
        Base configuration to start from.  When ``None`` a fresh
        :class:`RateConfig` instance is used.
    """

    config = defaults or RateConfig()

    if path is None:
        cwd = Path.cwd()
        candidates = [
            cwd / "configs" / "rates.yaml",
            cwd / "configs" / "rates.yml",
            cwd / "configs" / "rates.json",
        ]
        for candidate in candidates:
            if candidate.exists():
                path = candidate
                break

    overrides: dict[str, float] = {}
    if path is not None:
        file_overrides = _load_rates_from_file(Path(path))
        overrides.update(file_overrides)

    env_overrides = _load_rates_from_env(env_prefix)
    overrides.update(env_overrides)

    if overrides:
        config = config.with_overrides(**overrides)

    return config


def _load_rates_from_file(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}

    raw: Mapping[str, Any]
    if path.suffix.lower() in {".yaml", ".yml"}:
        if yaml is None:
            raise RuntimeError(
                "PyYAML is required to load YAML rate configs but is not "
                "installed."
            )
        with path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
    else:
        with path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle) or {}

    if not isinstance(raw, Mapping):
        raise ValueError(
            "Rate configuration file '{path}' must define a mapping, got "
            f"{type(raw)}."
        )

    overrides: dict[str, float] = {}
    for key, value in raw.items():
        resolved = _resolve_rate_value(value)
        if resolved is not None:
            overrides[key.lower()] = resolved
    return overrides


def _load_rates_from_env(prefix: str) -> dict[str, float]:
    overrides: dict[str, float] = {}
    for key, raw_value in os.environ.items():
        if not key.startswith(prefix):
            continue
        clean_key = key[len(prefix) :].lower()
        resolved = _resolve_rate_value(raw_value)
        if resolved is not None:
            overrides[clean_key] = resolved
    return overrides


def _resolve_rate_value(value: Any) -> float | None:
    if value is None:
        return None

    if isinstance(value, Mapping):
        if "dt" in value:
            return float(value["dt"])
        if "hz" in value:
            hz = float(value["hz"])
            if hz <= 0:
                raise ValueError("Frequency overrides must be positive.")
            return 1.0 / hz
        raise ValueError(
            "Rate mapping overrides must contain either 'dt' or 'hz'."
        )

    if isinstance(value, (list, tuple)) and value:
        return _resolve_rate_value(value[0])

    if isinstance(value, str):
        stripped = value.strip().lower()
        if stripped.endswith("hz"):
            hz = float(stripped[:-2])
            if hz <= 0:
                raise ValueError("Frequency overrides must be positive.")
            return 1.0 / hz
        if stripped.endswith("ms"):
            ms = float(stripped[:-2])
            if ms <= 0:
                raise ValueError("Millisecond overrides must be positive.")
            return ms / 1000.0
        return float(stripped)

    if isinstance(value, _Number):
        return float(value)

    raise TypeError(
        "Unsupported rate override type. Expected mapping, string, or number; "
        f"got {type(value)}."
    )


__all__ = ["RateConfig", "load_rate_config"]
