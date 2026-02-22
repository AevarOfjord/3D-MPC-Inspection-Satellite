"""
Configuration I/O Module.

Provides save/load functionality for SimulationConfig objects.
Supports YAML and JSON formats with versioned schemas and migration helpers.

Usage:
    from config.io import ConfigIO

    # Save config
    config = SimulationConfig.create_default()
    ConfigIO.save(config, "config.yaml")

    # Load config
    loaded_config = ConfigIO.load("config.yaml")

    # Migrate old config
    migrated_config = ConfigIO.migrate("old_config.json", target_version="3.0.0")
"""

import json
import logging
from pathlib import Path
from typing import Any

try:
    import yaml

    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

from .mission_state import DEFAULT_PATH_HOLD_END_S, MissionState
from .models import AppConfig
from .simulation_config import SimulationConfig

logger = logging.getLogger(__name__)

# Current config schema version
CURRENT_CONFIG_VERSION = "3.0.0"


class ConfigIO:
    """
    Configuration I/O handler for SimulationConfig objects.

    Supports saving/loading configurations in YAML and JSON formats,
    with versioned schemas and migration support.
    """

    @staticmethod
    def save(
        config: SimulationConfig,
        file_path: str | Path,
        format: str = "auto",
        include_metadata: bool = True,
    ) -> None:
        """
        Save SimulationConfig to file.

        Args:
            config: SimulationConfig to save
            file_path: Path to output file (.yaml, .yml, or .json)
            format: File format ("yaml", "json", or "auto" to detect from extension)
            include_metadata: Include version and timestamp metadata

        Raises:
            ValueError: If format is invalid or YAML not available
            IOError: If file cannot be written
        """
        file_path = Path(file_path)

        # Determine format
        if format == "auto":
            if file_path.suffix.lower() in (".yaml", ".yml"):
                format = "yaml"
            elif file_path.suffix.lower() == ".json":
                format = "json"
            else:
                # Default to YAML if available, otherwise JSON
                format = "yaml" if YAML_AVAILABLE else "json"

        if format == "yaml" and not YAML_AVAILABLE:
            raise ValueError(
                "YAML format requested but PyYAML not installed. "
                "Install with: pip install pyyaml"
            )

        # Convert config to dictionary
        config_dict = ConfigIO._config_to_dict(config, include_metadata)

        # Write to file
        try:
            if format == "yaml":
                with open(file_path, "w") as f:
                    yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)
            else:  # JSON
                with open(file_path, "w") as f:
                    json.dump(config_dict, f, indent=2, sort_keys=False)

            logger.info(f"Configuration saved to {file_path}")
        except OSError as e:
            raise OSError(f"Failed to write config file {file_path}: {e}") from e

    @staticmethod
    def load(
        file_path: str | Path,
        migrate: bool = True,
    ) -> SimulationConfig:
        """
        Load SimulationConfig from file.

        Args:
            file_path: Path to config file (.yaml, .yml, or .json)
            migrate: Automatically migrate older config versions if True

        Returns:
            SimulationConfig loaded from file

        Raises:
            FileNotFoundError: If file doesn't exist
            ValueError: If file format is invalid or migration fails
        """
        file_path = Path(file_path)

        if not file_path.exists():
            raise FileNotFoundError(f"Config file not found: {file_path}")

        # Detect format and load
        if file_path.suffix.lower() in (".yaml", ".yml"):
            if not YAML_AVAILABLE:
                raise ValueError(
                    "YAML file detected but PyYAML not installed. "
                    "Install with: pip install pyyaml"
                )
            with open(file_path) as f:
                config_dict = yaml.safe_load(f)
        elif file_path.suffix.lower() == ".json":
            with open(file_path) as f:
                config_dict = json.load(f)
        else:
            raise ValueError(
                f"Unsupported file format: {file_path.suffix}. "
                "Use .yaml, .yml, or .json"
            )

        if not isinstance(config_dict, dict):
            raise ValueError(
                f"Invalid config file format: expected dict, got {type(config_dict)}"
            )

        # Check version and migrate if needed
        file_version = config_dict.get("_metadata", {}).get("version", "1.0.0")
        if migrate and file_version != CURRENT_CONFIG_VERSION:
            logger.info(
                f"Migrating config from version {file_version} to {CURRENT_CONFIG_VERSION}"
            )
            config_dict = ConfigIO._migrate_config(
                config_dict, file_version, CURRENT_CONFIG_VERSION
            )

        # Convert dict to SimulationConfig
        return ConfigIO._dict_to_config(config_dict)

    @staticmethod
    def migrate(
        config_dict: dict[str, Any],
        source_version: str,
        target_version: str = CURRENT_CONFIG_VERSION,
    ) -> dict[str, Any]:
        """
        Migrate configuration dictionary between versions.

        Args:
            config_dict: Configuration dictionary to migrate
            source_version: Source version (e.g., "2.0.0")
            target_version: Target version (default: current)

        Returns:
            Migrated configuration dictionary
        """
        if source_version == target_version:
            return config_dict

        return ConfigIO._migrate_config(config_dict, source_version, target_version)

    @staticmethod
    def _config_to_dict(
        config: SimulationConfig,
        include_metadata: bool = True,
    ) -> dict[str, Any]:
        """Convert SimulationConfig to dictionary."""
        result = {
            "app_config": config.app_config.model_dump(),
            "mission_state": ConfigIO._mission_state_to_dict(config.mission_state),
        }

        if include_metadata:
            import datetime

            result["_metadata"] = {
                "version": CURRENT_CONFIG_VERSION,
                "created_at": datetime.datetime.now().isoformat(),
                "description": "Satellite Control System Configuration",
            }

        return result

    @staticmethod
    def _dict_to_config(config_dict: dict[str, Any]) -> SimulationConfig:
        """Convert dictionary to SimulationConfig."""
        # Remove metadata if present
        config_dict = {k: v for k, v in config_dict.items() if not k.startswith("_")}

        # Create AppConfig
        app_config = AppConfig(**config_dict.get("app_config", {}))

        # Create MissionState
        mission_state_dict = config_dict.get("mission_state", {})
        mission_state = ConfigIO._dict_to_mission_state(mission_state_dict)

        return SimulationConfig(
            app_config=app_config,
            mission_state=mission_state,
        )

    @staticmethod
    def _mission_state_to_dict(mission_state: MissionState) -> dict[str, Any]:
        """Convert MissionState to dictionary (path runtime only)."""
        result: dict[str, Any] = {
            "path": {
                "active": bool(mission_state.path.active),
                "waypoints": list(mission_state.path.waypoints),
                "path_speed": float(mission_state.path.path_speed),
                "path_length": float(mission_state.path.path_length),
            },
            "path_hold_end": float(mission_state.path_hold_end),
        }

        path_tracking_keys = (
            "path_tracking_center",
            "path_tracking_base_shape",
            "path_tracking_phase",
            "path_tracking_closest_point_index",
            "path_tracking_estimated_duration",
            "path_tracking_mission_start_time",
            "path_tracking_tracking_start_time",
            "path_tracking_positioning_start_time",
            "path_tracking_stabilization_start_time",
            "path_tracking_current_target_position",
            "path_tracking_final_position",
            "path_tracking_target_start_distance",
            "path_tracking_has_return",
            "path_tracking_return_position",
            "path_tracking_return_angle",
            "path_tracking_trajectory",
            "path_tracking_trajectory_dt",
        )
        for key in path_tracking_keys:
            result[key] = getattr(mission_state, key)

        return result

    @staticmethod
    def _dict_to_mission_state(mission_state_dict: dict[str, Any]) -> MissionState:
        """Convert dictionary to MissionState using canonical runtime fields."""
        from .mission_state import (
            MissionState,
            PathFollowingState,
        )

        def _resolve_path_hold_end() -> float:
            hold_raw = mission_state_dict.get("path_hold_end")
            if hold_raw is None:
                return float(DEFAULT_PATH_HOLD_END_S)
            return float(hold_raw)

        def _apply_path_tracking_fields(ms: MissionState) -> MissionState:
            path_tracking_keys = (
                "path_tracking_center",
                "path_tracking_base_shape",
                "path_tracking_phase",
                "path_tracking_closest_point_index",
                "path_tracking_estimated_duration",
                "path_tracking_mission_start_time",
                "path_tracking_tracking_start_time",
                "path_tracking_positioning_start_time",
                "path_tracking_stabilization_start_time",
                "path_tracking_current_target_position",
                "path_tracking_final_position",
                "path_tracking_target_start_distance",
                "path_tracking_has_return",
                "path_tracking_return_position",
                "path_tracking_return_angle",
                "path_tracking_trajectory",
                "path_tracking_trajectory_dt",
            )
            for key in path_tracking_keys:
                if key in mission_state_dict:
                    setattr(ms, key, mission_state_dict[key])
            return ms

        ms = MissionState(
            path=PathFollowingState(**mission_state_dict.get("path", {})),
            path_hold_end=_resolve_path_hold_end(),
        )
        return _apply_path_tracking_fields(ms)

    @staticmethod
    def _migrate_config(
        config_dict: dict[str, Any],
        source_version: str,
        target_version: str,
    ) -> dict[str, Any]:
        """
        Migrate configuration between versions.

        Currently supports:
        - 1.0.0 -> 2.0.0: Add mission_state structure
        - 2.0.0 -> 3.0.0: Move timing parameters to SimulationParams
        """
        version_parts = source_version.split(".")
        major = int(version_parts[0])
        minor = int(version_parts[1]) if len(version_parts) > 1 else 0

        target_parts = target_version.split(".")
        target_major = int(target_parts[0])
        target_minor = int(target_parts[1]) if len(target_parts) > 1 else 0

        # Migrate step by step
        current_dict = config_dict.copy()
        current_version = (major, minor)
        target_version_tuple = (target_major, target_minor)

        # 1.0.0 -> 2.0.0: Add mission_state if missing
        if current_version < (2, 0):
            if "mission_state" not in current_dict:
                logger.info("Migrating from v1.0.0: Adding mission_state structure")
                current_dict["mission_state"] = {}
            current_version = (2, 0)

        # 2.0.0 -> 3.0.0: Move timing parameters to SimulationParams
        if current_version < (3, 0) and target_version_tuple >= (3, 0):
            logger.info(
                "Migrating from v2.0.0: Moving timing parameters to SimulationParams"
            )
            app_config = current_dict.get("app_config", {})
            simulation = app_config.get("simulation", {})

            # Add timing parameters if not present (use defaults)
            if "control_dt" not in simulation:
                simulation["control_dt"] = 0.050

            if "default_path_speed" not in simulation:
                simulation["default_path_speed"] = 0.1

            app_config["simulation"] = simulation
            current_dict["app_config"] = app_config
            current_version = (3, 0)

        # Update metadata version
        if "_metadata" in current_dict:
            current_dict["_metadata"]["version"] = target_version
            current_dict["_metadata"]["migrated_from"] = source_version

        return current_dict

    @staticmethod
    def validate(config_dict: dict[str, Any]) -> tuple[bool, list[str]]:
        """
        Validate configuration dictionary structure.

        Args:
            config_dict: Configuration dictionary to validate

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        # Check required top-level keys
        if "app_config" not in config_dict:
            errors.append("Missing required key: app_config")
        if "mission_state" not in config_dict:
            errors.append("Missing required key: mission_state")

        # Validate app_config structure
        if "app_config" in config_dict:
            app_config = config_dict["app_config"]
            required_sections = ["physics", "mpc", "simulation"]
            for section in required_sections:
                if section not in app_config:
                    errors.append(f"Missing app_config section: {section}")

        # Try to create config objects to validate structure
        if not errors:
            try:
                ConfigIO._dict_to_config(config_dict)
            except Exception as e:
                errors.append(f"Invalid config structure: {e}")

        return len(errors) == 0, errors
