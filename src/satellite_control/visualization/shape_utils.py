"""
Shape Utilities for Visualization

Provides utilities for working with geometric shapes, DXF files, and path transformations.
Extracted from unified_visualizer.py to improve modularity.
"""

from typing import List, Tuple


def get_demo_shape(shape_type: str) -> List[Tuple[float, float, float]]:
    """
    Get predefined demo shape points (matches Mission.py implementation).
    
    Args:
        shape_type: Type of shape ('rectangle', 'triangle', 'hexagon')
        
    Returns:
        List of (x, y, z) tuples defining the shape
    """
    import math
    import numpy as np
    
    if shape_type == "rectangle":
        # 0.4m x 0.3m rectangle centered at origin
        return [
            (-0.2, -0.15, 0.0),
            (0.2, -0.15, 0.0),
            (0.2, 0.15, 0.0),
            (-0.2, 0.15, 0.0),
            (-0.2, -0.15, 0.0),  # Close the shape
        ]
    elif shape_type == "triangle":
        # Equilateral triangle with 0.4m sides
        return [
            (0.0, 0.2, 0.0),
            (-0.173, -0.1, 0.0),
            (0.173, -0.1, 0.0),
            (0.0, 0.2, 0.0),
        ]
    elif shape_type == "hexagon":
        # Regular hexagon with 0.2m radius
        points = []
        for i in range(7):  # 7 points to close the shape
            angle = i * np.pi / 3
            x = 0.2 * np.cos(angle)
            y = 0.2 * np.sin(angle)
            points.append((x, y, 0.0))
        return points
    else:
        # Default to rectangle
        return get_demo_shape("rectangle")


def transform_shape(
    points: List[Tuple[float, ...]],
    center: Tuple[float, ...],
    rotation: float,
) -> List[Tuple[float, float, float]]:
    """
    Transform shape points to specified center and rotation.
    
    Args:
        points: List of (x, y, z) tuples
        center: Translation center (x, y, z)
        rotation: Rotation angle in radians
        
    Returns:
        Transformed list of (x, y, z) tuples
    """
    import numpy as np
    
    cos_r = np.cos(rotation)
    sin_r = np.sin(rotation)
    
    cx = center[0]
    cy = center[1]
    cz = center[2] if len(center) > 2 else 0.0

    transformed = []
    for p in points:
        x = p[0]
        y = p[1]
        z = p[2] if len(p) > 2 else 0.0

        # Rotate (XY plane)
        x_rot = x * cos_r - y * sin_r
        y_rot = x * sin_r + y * cos_r

        # Translate
        x_final = x_rot + cx
        y_final = y_rot + cy
        z_final = z + cz

        transformed.append((x_final, y_final, z_final))
    
    return transformed


def make_offset_path(
    points: List[Tuple[float, ...]], offset_distance: float
) -> List[Tuple[float, float, float]]:
    """
    Create an outward offset path (matches Mission.py implementation).
    
    Args:
        points: List of (x, y, z) tuples defining the shape
        offset_distance: Distance to offset the path in meters
        
    Returns:
        List of (x, y, z) tuples defining the offset path
    """
    import numpy as np
    
    if len(points) < 3:
        return [(p[0], p[1], p[2] if len(p) > 2 else 0.0) for p in points]

    points_2d = [(p[0], p[1]) for p in points]
    zs = [p[2] for p in points if len(p) > 2]
    z_level = float(np.mean(zs)) if zs else 0.0
    
    # Try to use DXF_Viewer's make_offset_path if available
    try:
        from DXF.dxf_viewer import make_offset_path as viewer_make_offset_path
        
        offset_2d = viewer_make_offset_path(
            points_2d,
            float(offset_distance),
            1.0,
            join="round",
            resolution=24,
            mode="buffer",
        )
        return [(p[0], p[1], z_level) for p in offset_2d]
    except Exception:
        pass
    
    # Fallback: simple centroid-based offset
    pts = points_2d[:]
    if np.linalg.norm(np.array(pts[0]) - np.array(pts[-1])) > 1e-8:
        pts = pts + [pts[0]]
    
    # Calculate shape centroid
    centroid_x = np.mean([p[0] for p in pts])
    centroid_y = np.mean([p[1] for p in pts])
    centroid = np.array([centroid_x, centroid_y])
    
    upscaled = []
    for i in range(len(pts) - 1):
        current = np.array(pts[i])
        next_point = np.array(pts[i + 1])
        
        edge_vec = next_point - current
        edge_normal = np.array([-edge_vec[1], edge_vec[0]])
        if np.linalg.norm(edge_normal) > 0:
            edge_normal = edge_normal / np.linalg.norm(edge_normal)
        
        mid_point = (current + next_point) / 2
        to_mid = mid_point - centroid
        if np.dot(edge_normal, to_mid) < 0:
            edge_normal = -edge_normal
        
        offset_current = current + edge_normal * offset_distance
        upscaled.append((float(offset_current[0]), float(offset_current[1]), z_level))
    
    if np.linalg.norm(np.array(pts[0]) - np.array(pts[-1])) < 1e-6:
        upscaled.append(upscaled[0])
    
    return upscaled


def load_dxf_shape(dxf_path: str) -> List[Tuple[float, float, float]]:
    """
    Load shape points from DXF file using DXF_Viewer pipeline.
    
    Matches the DXF loading logic used in Mission.py for consistency.
    
    Args:
        dxf_path: Path to DXF file
        
    Returns:
        List of (x, y, z) tuples in meters
        
    Raises:
        ImportError: If DXF viewer utilities are unavailable
        ValueError: If no usable boundary could be constructed
    """
    import ezdxf
    
    try:
        from DXF.dxf_viewer import (
            extract_boundary_polygon,
            sanitize_boundary,
            units_code_to_name_and_scale,
        )
    except Exception as e:
        raise ImportError(f"dxf_viewer utilities unavailable: {e}")
    
    # Read DXF and determine units
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    insunits = int(doc.header.get("$INSUNITS", 0))
    units_name, to_m = units_code_to_name_and_scale(insunits)
    
    # Extract and sanitize boundary in native units, then scale to meters
    boundary = extract_boundary_polygon(msp)
    boundary = sanitize_boundary(boundary, to_m)
    boundary_m = (
        [(float(x) * to_m, float(y) * to_m, 0.0) for (x, y) in boundary]
        if boundary
        else []
    )
    
    if not boundary_m:
        raise ValueError("No usable DXF boundary could be constructed.")
    
    print(f" DXF Loaded: {len(boundary_m)} points")
    print(f"   Units: {units_name} (INSUNITS={insunits}), scaled → meters (x{to_m})")
    return boundary_m
