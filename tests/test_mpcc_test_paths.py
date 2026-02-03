import numpy as np

from src.satellite_control.mission.mpcc_test_paths import MPCC_TEST_CASES, build_test_path


def test_mpcc_test_cases_generate_paths():
    """Ensure each MPCC test path is well-formed and non-trivial."""
    for key in MPCC_TEST_CASES:
        path, length, speed = build_test_path(key)
        assert len(path) > 5
        assert length > 0.0
        assert speed > 0.0
        p0 = np.array(path[0], dtype=float)
        p1 = np.array(path[-1], dtype=float)
        assert p0.shape[0] == 3
        assert p1.shape[0] == 3


def test_mpcc_test_cases_complexity_metrics():
    """Check that complex paths have non-trivial curvature/variation."""
    for key in MPCC_TEST_CASES:
        path, length, _ = build_test_path(key)
        pts = np.array(path, dtype=float)
        start = pts[0]
        end = pts[-1]
        direct = float(np.linalg.norm(end - start))
        assert length >= direct

        # Some cases are intentionally loopy; require extra length.
        if key in {"figure_eight", "spiral_inward", "lissajous", "clover", "helix_wave"}:
            assert length >= max(1.0, direct * 1.2)

        # Ensure 3D variation exists for select cases.
        if key in {"helical_arc", "s_curve", "spiral_inward", "lissajous", "helix_wave"}:
            z_span = float(np.ptp(pts[:, 2]))
            assert z_span > 0.05
