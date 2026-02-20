"""Integration tests for scan project save/load and compile pipeline."""

from __future__ import annotations

import re
from pathlib import Path

from fastapi.testclient import TestClient
from satellite_control.dashboard.app import app

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCAN_PROJECT_DIR = PROJECT_ROOT / 'assets' / 'scan_projects'


def _safe_id(name: str) -> str:
    raw = name.strip() or 'scan_project'
    safe = re.sub(r'[^A-Za-z0-9._-]+', '_', raw).strip('_')
    return safe or 'scan_project'


def _default_scan_project(name: str) -> dict:
    return {
        'schema_version': 1,
        'name': name,
        'obj_path': 'assets/model_files/ISS/ISS.obj',
        'path_density_multiplier': 1.0,
        'scans': [
            {
                'id': 'scan_1',
                'name': 'Scan 1',
                'axis': 'Z',
                'plane_a': [0.0, 0.0, -0.5],
                'plane_b': [0.0, 0.0, 0.5],
                'turns': 4.0,
                'coarse_points_per_turn': 4,
                'densify_multiplier': 3,
                'speed_max': 0.2,
                'key_levels': [
                    {
                        'id': 'kl_1',
                        't': 0.0,
                        'center_offset': [0.0, 0.0],
                        'radius_x': 1.0,
                        'radius_y': 1.0,
                        'rotation_deg': 0.0,
                    },
                    {
                        'id': 'kl_2',
                        't': 1.0,
                        'center_offset': [0.0, 0.0],
                        'radius_x': 1.2,
                        'radius_y': 0.8,
                        'rotation_deg': 10.0,
                    },
                ],
            }
        ],
        'connectors': [],
    }


def test_scan_project_roundtrip_and_compile_single_scan():
    name = 'TestScanProject_Single'
    project_path = SCAN_PROJECT_DIR / f'{_safe_id(name)}.json'

    with TestClient(app) as client:
        payload = _default_scan_project(name)
        save_resp = client.post('/scan_projects', json=payload)
        assert save_resp.status_code == 200
        saved = save_resp.json()
        assert saved['name'] == name
        assert saved['id'] == _safe_id(name)

        list_resp = client.get('/scan_projects')
        assert list_resp.status_code == 200
        projects = list_resp.json().get('projects', [])
        assert any(item.get('id') == saved['id'] for item in projects)

        load_resp = client.get(f"/scan_projects/{saved['id']}")
        assert load_resp.status_code == 200
        loaded = load_resp.json()
        assert loaded['name'] == name
        assert len(loaded['scans']) == 1

        compile_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': loaded,
                'quality': 'preview',
                'include_collision': True,
                'collision_threshold_m': 0.05,
            },
        )
        assert compile_resp.status_code == 200
        compiled = compile_resp.json()
        assert compiled['status'] == 'success'
        assert compiled['points'] > 0
        assert compiled['path_length'] > 0
        assert 'scan_1' in compiled['endpoints']
        assert 'diagnostics' in compiled
        assert 'collision_points_count' in compiled['diagnostics']

    if project_path.exists():
        project_path.unlink()


def test_scan_project_compile_multi_scan_chain_validation_and_collision_fields():
    name = 'TestScanProject_Multi'
    project_path = SCAN_PROJECT_DIR / f'{_safe_id(name)}.json'

    with TestClient(app) as client:
        project = _default_scan_project(name)
        scan2 = {
            'id': 'scan_2',
            'name': 'Scan 2',
            'axis': 'X',
            'plane_a': [-0.3, 0.0, 0.0],
            'plane_b': [0.6, 0.0, 0.0],
            'turns': 3.0,
            'coarse_points_per_turn': 4,
            'densify_multiplier': 2,
            'speed_max': 0.25,
            'key_levels': [
                {
                    'id': 'kl_3',
                    't': 0.0,
                    'center_offset': [0.0, 0.0],
                    'radius_x': 0.8,
                    'radius_y': 0.8,
                    'rotation_deg': 0.0,
                },
                {
                    'id': 'kl_4',
                    't': 1.0,
                    'center_offset': [0.1, -0.1],
                    'radius_x': 1.0,
                    'radius_y': 0.7,
                    'rotation_deg': -5.0,
                },
            ],
        }
        project['scans'].append(scan2)

        # Missing connectors are allowed for preview so users can inspect
        # independent scans before connecting them.
        preview_compile = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'preview',
                'include_collision': True,
            },
        )
        assert preview_compile.status_code == 200
        preview_payload = preview_compile.json()
        assert preview_payload['status'] == 'success'
        assert len(preview_payload['scan_paths']) == 2

        # Missing connectors must still fail for final bake.
        bad_compile = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'final',
                'include_collision': True,
            },
        )
        assert bad_compile.status_code == 400

        project['connectors'] = [
            {
                'id': 'conn_1',
                'from_scan_id': 'scan_1',
                'to_scan_id': 'scan_2',
                'from_endpoint': 'end',
                'to_endpoint': 'start',
                'samples': 24,
            }
        ]

        save_resp = client.post('/scan_projects', json=project)
        assert save_resp.status_code == 200
        saved = save_resp.json()

        ok_compile = client.post(
            '/scan_projects/compile',
            json={
                'project': saved,
                'quality': 'final',
                'include_collision': True,
                'collision_threshold_m': 0.05,
            },
        )
        assert ok_compile.status_code == 200
        compiled = ok_compile.json()
        assert compiled['status'] == 'success'
        assert len(compiled['scan_paths']) == 2
        assert len(compiled['connector_paths']) == 1
        assert compiled['diagnostics']['clearance_threshold_m'] == 0.05
        assert isinstance(compiled['diagnostics']['warnings'], list)

    if project_path.exists():
        project_path.unlink()


def test_scan_project_compile_translation_preserves_centerline_shift():
    with TestClient(app) as client:
        base = _default_scan_project('TestScanProject_Translation_Base')
        base['scans'][0]['axis'] = 'X'
        base['scans'][0]['plane_a'] = [-0.5, 0.0, 0.0]
        base['scans'][0]['plane_b'] = [0.5, 0.0, 0.0]
        base['scans'][0]['turns'] = 3.0

        shifted = _default_scan_project('TestScanProject_Translation_Shifted')
        shifted['scans'][0]['axis'] = 'X'
        delta = [2.5, -1.2, 0.7]
        shifted['scans'][0]['plane_a'] = [
            base['scans'][0]['plane_a'][0] + delta[0],
            base['scans'][0]['plane_a'][1] + delta[1],
            base['scans'][0]['plane_a'][2] + delta[2],
        ]
        shifted['scans'][0]['plane_b'] = [
            base['scans'][0]['plane_b'][0] + delta[0],
            base['scans'][0]['plane_b'][1] + delta[1],
            base['scans'][0]['plane_b'][2] + delta[2],
        ]
        shifted['scans'][0]['turns'] = 3.0

        base_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': base,
                'quality': 'preview',
                'include_collision': False,
            },
        )
        assert base_resp.status_code == 200
        base_path = base_resp.json()['combined_path']
        assert len(base_path) > 2

        shifted_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': shifted,
                'quality': 'preview',
                'include_collision': False,
            },
        )
        assert shifted_resp.status_code == 200
        shifted_path = shifted_resp.json()['combined_path']
        assert len(shifted_path) == len(base_path)

        def _centroid(path: list[list[float]]) -> list[float]:
            n = float(len(path))
            return [
                sum(p[0] for p in path) / n,
                sum(p[1] for p in path) / n,
                sum(p[2] for p in path) / n,
            ]

        c0 = _centroid(base_path)
        c1 = _centroid(shifted_path)
        actual_delta = [c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2]]

        tol = 1e-4
        assert abs(actual_delta[0] - delta[0]) <= tol
        assert abs(actual_delta[1] - delta[1]) <= tol
        assert abs(actual_delta[2] - delta[2]) <= tol


def test_scan_project_preview_connector_respects_selected_endpoint_direction():
    with TestClient(app) as client:
        project = _default_scan_project('TestScanProject_ConnectorPreviewDirection')
        project['scans'].append(
            {
                'id': 'scan_2',
                'name': 'Scan 2',
                'axis': 'X',
                'plane_a': [-0.3, 0.0, 0.0],
                'plane_b': [0.6, 0.0, 0.0],
                'turns': 3.0,
                'coarse_points_per_turn': 4,
                'densify_multiplier': 2,
                'speed_max': 0.25,
                'key_levels': [
                    {
                        'id': 'kl_3',
                        't': 0.0,
                        'center_offset': [0.0, 0.0],
                        'radius_x': 0.8,
                        'radius_y': 0.8,
                        'rotation_deg': 0.0,
                    },
                    {
                        'id': 'kl_4',
                        't': 1.0,
                        'center_offset': [0.1, -0.1],
                        'radius_x': 1.0,
                        'radius_y': 0.7,
                        'rotation_deg': -5.0,
                    },
                ],
            }
        )

        baseline_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'preview',
                'include_collision': False,
            },
        )
        assert baseline_resp.status_code == 200
        baseline = baseline_resp.json()
        expected_start = baseline['endpoints']['scan_2']['start']
        expected_end = baseline['endpoints']['scan_1']['end']

        project['connectors'] = [
            {
                'id': 'conn_reverse_click_order',
                'from_scan_id': 'scan_2',
                'to_scan_id': 'scan_1',
                'from_endpoint': 'start',
                'to_endpoint': 'end',
                'samples': 24,
            }
        ]

        compile_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'preview',
                'include_collision': False,
            },
        )
        assert compile_resp.status_code == 200
        payload = compile_resp.json()
        assert payload['status'] == 'success'
        assert len(payload['connector_paths']) == 1

        connector_path = payload['connector_paths'][0]['path']
        assert len(connector_path) >= 2

        def _distance(a: list[float], b: list[float]) -> float:
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5

        assert _distance(connector_path[0], expected_start) < 1e-6
        assert _distance(connector_path[-1], expected_end) < 1e-6


def test_scan_project_path_density_multiplier_scales_points():
    with TestClient(app) as client:
        project = _default_scan_project('TestScanProject_DensityScale')
        project['path_density_multiplier'] = 0.5

        low_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'preview',
                'include_collision': False,
            },
        )
        assert low_resp.status_code == 200
        low_payload = low_resp.json()

        project['path_density_multiplier'] = 2.0
        high_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'preview',
                'include_collision': False,
            },
        )
        assert high_resp.status_code == 200
        high_payload = high_resp.json()

        # Same density policy for preview/final point generation.
        final_resp = client.post(
            '/scan_projects/compile',
            json={
                'project': project,
                'quality': 'final',
                'include_collision': False,
            },
        )
        assert final_resp.status_code == 200
        final_payload = final_resp.json()

        assert high_payload['points'] > low_payload['points']
        assert final_payload['points'] == high_payload['points']
