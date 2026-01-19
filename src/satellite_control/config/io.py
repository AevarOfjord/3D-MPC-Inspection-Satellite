"""
Configuration I/O Module (V3.0.0)

Provides save/load functionality for SimulationConfig objects.
Supports YAML and JSON formats with versioned schemas and migration helpers.

Usage:
    from src.satellite_control.config.io import ConfigIO

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
from typing import Any, Dict, Union

try:
    import yaml

    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

from .models import AppConfig
from .mission_state import MissionState
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
        file_path: Union[str, Path],
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
        except IOError as e:
            raise IOError(f"Failed to write config file {file_path}: {e}") from e

    @staticmethod
    def load(
        file_path: Union[str, Path],
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
            with open(file_path, "r") as f:
                config_dict = yaml.safe_load(f)
        elif file_path.suffix.lower() == ".json":
            with open(file_path, "r") as f:
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
        config_dict: Dict[str, Any],
        source_version: str,
        target_version: str = CURRENT_CONFIG_VERSION,
    ) -> Dict[str, Any]:
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
    ) -> Dict[str, Any]:
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
    def _dict_to_config(config_dict: Dict[str, Any]) -> SimulationConfig:
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
    def _mission_state_to_dict(mission_state: MissionState) -> Dict[str, Any]:
        """Convert MissionState to dictionary."""
        from dataclasses import asdict

        return asdict(mission_state)

    @staticmethod
    def _dict_to_mission_state(mission_state_dict: Dict[str, Any]) -> MissionState:
        """Convert dictionary to MissionState, handling both flat(legacy) and nested structures."""
        from .mission_state import (
            MissionState,
            WaypointState,
            ShapeFollowingState,
            ScanState,
            TrajectoryState,
            ObstacleState,
        )

        # 1. Attempt to load as new nested structure first
        if "waypoint" in mission_state_dict or "shape" in mission_state_dict:
            # Looks like new structure
            return MissionState(
                waypoint=WaypointState(**mission_state_dict.get("waypoint", {})),
                shape=ShapeFollowingState(**mission_state_dict.get("shape", {})),
                scan=ScanState(**mission_state_dict.get("scan", {})),
                trajectory=TrajectoryState(**mission_state_dict.get("trajectory", {})),
                obstacle_state=ObstacleState(
                    **mission_state_dict.get("obstacle_state", {})
                ),
            )

        # 2. Fallback: Map legacy flat keys to new structure
        ms = MissionState()

        # Waypoint
        ms.waypoint.enabled = mission_state_dict.get(
            "enable_waypoint_mode", False
        ) or mission_state_dict.get("enable_multi_point_mode", False)
        ms.waypoint.targets = mission_state_dict.get(
            "waypoint_targets", []
        ) or mission_state_dict.get("multi_point_targets", [])
        ms.waypoint.angles = mission_state_dict.get(
            "waypoint_angles", []
        ) or mission_state_dict.get("multi_point_angles", [])
        ms.waypoint.current_target_index = mission_state_dict.get(
            "current_target_index", 0
        )
        ms.waypoint.start_time = mission_state_dict.get(
            "target_stabilization_start_time"
        )
        ms.waypoint.phase = mission_state_dict.get(
            "waypoint_phase"
        ) or mission_state_dict.get("multi_point_phase")

        # Shape
        ms.shape.active = mission_state_dict.get("dxf_shape_mode_active", False)
        ms.shape.center = mission_state_dict.get("dxf_shape_center")
        ms.shape.path = mission_state_dict.get("dxf_shape_path", [])
        ms.shape.base_shape = mission_state_dict.get("dxf_base_shape", [])
        ms.shape.target_speed = mission_state_dict.get("dxf_target_speed", 0.1)
        ms.shape.estimated_duration = mission_state_dict.get(
            "dxf_estimated_duration", 60.0
        )
        ms.shape.phase = mission_state_dict.get("dxf_shape_phase", "POSITIONING")
        ms.shape.path_length = mission_state_dict.get("dxf_path_length", 0.0)
        ms.shape.closest_point_index = mission_state_dict.get(
            "dxf_closest_point_index", 0
        )
        ms.shape.current_target_position = mission_state_dict.get(
            "dxf_current_target_position"
        )
        ms.shape.tracking_start_time = mission_state_dict.get("dxf_tracking_start_time")
        ms.shape.offset_distance = mission_state_dict.get("dxf_offset_distance", 0.5)
        # Map other shape fields as needed...

        # Scan
        ms.scan.active = mission_state_dict.get("mesh_scan_mode_active", False)
        ms.scan.obj_path = mission_state_dict.get("mesh_scan_obj_path")
        # Map other scan fields...

        # Trajectory
        ms.trajectory.active = mission_state_dict.get("trajectory_mode_active", False)
        # Map other trajectory fields...

        # Obstacles
        ms.obstacle_state.enabled = mission_state_dict.get("obstacles_enabled", False)
        ms.obstacle_state.obstacles = mission_state_dict.get("obstacles", [])

        return ms

    @staticmethod
    def _migrate_config(
        config_dict: Dict[str, Any],
        source_version: str,
        target_version: str,
    ) -> Dict[str, Any]:
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
            if "target_hold_time" not in simulation:
                simulation["target_hold_time"] = 5.0
            if "waypoint_final_stabilization_time" not in simulation:
                simulation["waypoint_final_stabilization_time"] = 10.0
            if "shape_final_stabilization_time" not in simulation:
                simulation["shape_final_stabilization_time"] = 15.0
            if "shape_positioning_stabilization_time" not in simulation:
                simulation["shape_positioning_stabilization_time"] = 5.0
            if "default_target_speed" not in simulation:
                simulation["default_target_speed"] = 0.1

            app_config["simulation"] = simulation
            current_dict["app_config"] = app_config
            current_version = (3, 0)

        # Update metadata version
        if "_metadata" in current_dict:
            current_dict["_metadata"]["version"] = target_version
            current_dict["_metadata"]["migrated_from"] = source_version

        return current_dict

    @staticmethod
    def validate(config_dict: Dict[str, Any]) -> tuple[bool, list[str]]:
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
