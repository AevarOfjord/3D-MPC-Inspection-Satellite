"""
Mission Factory Module

Factory functions for creating various standard mission types.
Separates mission creation logic from data definitions.
"""

from typing import List
import numpy as np

from src.satellite_control.mission.mission_types import (
    Mission,
    MissionType,
    MissionPhase,
    Waypoint,
)


def create_flyby_mission(
    name: str = "Flyby Inspection",
    start_distance: float = 10.0,
    pass_distance: float = 3.0,
    approach_speed: float = 0.05,
) -> Mission:
    """
    Create a flyby mission template.

    Trajectory: Start -> Approach -> Flyby -> Depart
    """
    mission = Mission(
        name=name,
        mission_type=MissionType.FLYBY,
        description="Single-pass flyby inspection of target",
        start_position=np.array([start_distance, 0.0, 0.0]),
    )

    mission.phases = [
        MissionPhase(
            name="Approach",
            description="Approach target from starting position",
            waypoints=[
                Waypoint(
                    position=np.array([pass_distance + 2, 0.0, 0.0]),
                    approach_speed=approach_speed,
                    name="Approach point",
                ),
            ],
        ),
        MissionPhase(
            name="Flyby",
            description="Pass by target at close range",
            waypoints=[
                Waypoint(
                    position=np.array([pass_distance, 2.0, 0.0]),
                    approach_speed=approach_speed * 0.5,
                    name="Flyby start",
                ),
                Waypoint(
                    position=np.array([pass_distance, -2.0, 0.0]),
                    approach_speed=approach_speed * 0.5,
                    name="Flyby end",
                ),
            ],
        ),
        MissionPhase(
            name="Departure",
            description="Depart to safe distance",
            waypoints=[
                Waypoint(
                    position=np.array([start_distance, 0.0, 0.0]),
                    approach_speed=approach_speed,
                    name="Safe distance",
                ),
            ],
        ),
    ]

    return mission


def create_circumnavigation_mission(
    name: str = "Circumnavigation",
    orbit_radius: float = 5.0,
    num_points: int = 8,
    hold_time: float = 5.0,
) -> Mission:
    """
    Create a circumnavigation mission template.

    Orbit around target in XY plane with inspection holds.
    """
    mission = Mission(
        name=name,
        mission_type=MissionType.CIRCUMNAVIGATION,
        description="Complete orbit around target with inspection points",
        start_position=np.array([orbit_radius, 0.0, 0.0]),
    )

    # Generate circular waypoints
    waypoints = []
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points
        x = orbit_radius * np.cos(angle)
        y = orbit_radius * np.sin(angle)
        waypoints.append(
            Waypoint(
                position=np.array([x, y, 0.0]),
                hold_time=hold_time,
                approach_speed=0.03,
                name=f"Point {i + 1}",
            )
        )

    # Close the loop
    waypoints.append(
        Waypoint(
            position=np.array([orbit_radius, 0.0, 0.0]),
            hold_time=0,
            approach_speed=0.03,
            name="Return",
        )
    )

    mission.phases = [
        MissionPhase(
            name="Orbit",
            description="Circular inspection orbit",
            waypoints=waypoints,
        ),
    ]

    return mission


def create_station_keeping_mission(
    name: str = "Station Keeping",
    position: np.ndarray = None,
    duration: float = 300.0,
) -> Mission:
    """
    Create a station-keeping mission template.

    Hold position at specified offset from target.
    """
    if position is None:
        position = np.array([5.0, 0.0, 0.0])

    mission = Mission(
        name=name,
        mission_type=MissionType.STATION_KEEPING,
        description=f"Hold position at {position} for {duration}s",
        start_position=position.copy(),
    )

    mission.phases = [
        MissionPhase(
            name="Station Keeping",
            description="Maintain position",
            waypoints=[
                Waypoint(
                    position=position.copy(),
                    hold_time=duration,
                    approach_speed=0.02,
                    name="Hold position",
                ),
            ],
        ),
    ]

    return mission


def create_inspection_mission(
    name: str = "Detailed Inspection",
    inspection_points: List[np.ndarray] = None,
    hold_time: float = 30.0,
) -> Mission:
    """
    Create a multi-point inspection mission.

    Visit multiple inspection points with extended holds.
    """
    if inspection_points is None:
        # Default: 6 faces of target cube
        d = 3.0  # Distance from target
        inspection_points = [
            np.array([d, 0, 0]),  # +X
            np.array([0, d, 0]),  # +Y
            np.array([0, 0, d]),  # +Z
            np.array([-d, 0, 0]),  # -X
            np.array([0, -d, 0]),  # -Y
            np.array([0, 0, -d]),  # -Z
        ]

    mission = Mission(
        name=name,
        mission_type=MissionType.INSPECTION,
        description="Multi-point detailed inspection",
        start_position=np.array([5.0, 5.0, 0.0]),
    )

    waypoints = [
        Waypoint(
            position=pt.copy(),
            hold_time=hold_time,
            approach_speed=0.02,
            name=f"Inspection point {i + 1}",
        )
        for i, pt in enumerate(inspection_points)
    ]

    mission.phases = [
        MissionPhase(
            name="Inspection",
            description="Visit all inspection points",
            waypoints=waypoints,
        ),
        MissionPhase(
            name="Return",
            description="Return to safe distance",
            waypoints=[
                Waypoint(
                    position=np.array([5.0, 5.0, 0.0]),
                    hold_time=0,
                    approach_speed=0.05,
                    name="Safe standoff",
                ),
            ],
        ),
    ]

    return mission
