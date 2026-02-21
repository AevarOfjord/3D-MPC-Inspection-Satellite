# Changelog

## v4.0.0-beta.1 - 2026-02-18

### Added

- V4 guided planner UX with a new 3-column layout and step-focused context cards.
- Planner UX mode contract with persistence: `mission_control_planner_ux_mode_v1` (`guided` / `advanced`).
- Deterministic planner step status model (`locked | ready | complete | error`).
- Optional onboarding coachmarks with persistence: `mission_control_coachmarks_v1`.
- V4 UI primitives/tokens under `ui/src/components/ui-v4/` and `ui/src/styles/tokens.css`.
- V4 readiness e2e suite (`ui/tests/e2e/v4_readiness.spec.ts`) covering:
  - <= 5 minute mission authoring KPI flow
  - desktop layout checks at 1280/1440/1920
  - keyboard focus-visible and step navigation flow

### Changed

- Planner routes to `PlannerModeViewV4` by default in `ui/src/App.tsx`.
- Legacy planner runtime fallback path removed from app routing.
- Feedback contracts extended with:
  - dialog `form` intent
  - actionable toasts (`actionLabel`, `onAction`)
- Mission save flow now uses structured form dialog instead of plain prompt.
- Vite build warning noise reduced for large desktop 3D vendor chunks.

### Documentation

- Updated operator workflow docs in `docs/HOW_TO_USE.md` for V4 guided flow.
- Updated UI runtime behavior notes in `ui/README.md`.

## v1.0.0

- Initial baseline tracked in git history.
