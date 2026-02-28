# Mission API Compatibility Migration

This project keeps a compatibility layer for older mission flows while the canonical API remains under `/api/v2/missions/*`.

## Canonical Endpoints

- `POST /api/v2/missions/validate`
- `POST /api/v2/missions/preview`
- `POST /api/v2/missions`
- `GET /api/v2/missions`
- `GET /api/v2/missions/{mission_id}`
- `POST /api/v2/missions/drafts`
- `GET /api/v2/missions/drafts/list`
- `GET /api/v2/missions/drafts/{draft_id}`
- `POST /api/v2/missions/migrate_legacy`

## Compatibility Endpoints (Retained)

- `POST /mission_v2`
- `POST /mission_v2/preview`
- `GET /mission_v2`
- `POST /save_mission_v2`
- `GET /saved_missions_v2`
- `GET /mission_v2/{mission_name}`

These compatibility endpoints return deprecation headers and are retained so existing UI/test consumers continue to work during migration.

## Migration Guidance

1. Prefer `/api/v2/missions/*` endpoints for new development.
2. Use `POST /api/v2/missions/migrate_legacy` to normalize older mission payload envelopes.
3. Treat `/mission_v2*` and `/saved_missions_v2` as compatibility-only APIs.
