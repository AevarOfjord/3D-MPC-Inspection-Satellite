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
