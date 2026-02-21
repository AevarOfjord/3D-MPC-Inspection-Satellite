# Mission API V2 Migration Guide

This document defines the migration path from legacy mission endpoints to `/api/v2`.

## Scope

- Runtime target for this cycle: Python `3.11.x`
- Deployment model: local-first only (no hosted multi-user scope)
- Compatibility model: dual-stack for one minor cycle

## New Canonical Endpoints

Use these as the primary interface:

1. `POST /api/v2/missions/validate`
2. `POST /api/v2/missions/preview`
3. `POST /api/v2/missions`
4. `GET /api/v2/missions`
5. `GET /api/v2/missions/{mission_id}`
6. `POST /api/v2/missions/drafts`
7. `GET /api/v2/missions/drafts/{draft_id}`

## Payload Contract

`UnifiedMissionV2` requires:

- `schema_version` (must be `2`)
- `mission_id`
- `name`
- `epoch`
- `start_pose`
- `segments[]` (each segment has required `segment_id`)
- `metadata`

Validation responses use:

- `ValidationReportV2 = { valid, issues[], summary }`
- each issue includes `code`, `severity`, `path`, `message`, optional `suggestion`

## Legacy Compatibility

Legacy routes remain active temporarily:

- `/mission_v2`
- `/mission_v2/preview`
- `/save_mission_v2`
- `/saved_missions_v2`
- `/mission_v2/{mission_name}`

These now emit deprecation headers:

- `Deprecation: true`
- `Sunset: Mon, 18 May 2026 00:00:00 GMT`
- `Link: <this doc>; rel="deprecation"`

## Sunset Policy

Legacy endpoints are scheduled for removal on the earlier of:

1. `V2.1.0 + 30 days`
2. `V2.0.0 + 90 days`

Current default sunset header resolves to **May 18, 2026**.

## Migration Utility (CLI)

Use the converter script to migrate legacy mission JSON files:

```bash
.venv311/bin/python scripts/migrate_missions_v1_to_v2.py missions_unified --recursive --output-dir missions_v2_migrated --report-json release/migration-report.json
```text
In-place migration is supported when required:

```bash
.venv311/bin/python scripts/migrate_missions_v1_to_v2.py missions_unified --recursive --in-place
```text
## Validation Workflow Recommendation

1. Convert legacy missions using the CLI.
2. Run API validation: `POST /api/v2/missions/validate`.
3. Save through `POST /api/v2/missions`.
4. Use draft autosave endpoints from UI for crash-safe authoring.
