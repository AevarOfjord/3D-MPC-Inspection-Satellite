# Local User Ready Checklist

Goal: make this repo reliable for users who clone and run locally in a terminal.

## Phase 1 - Core Runtime Safety

- [x] Bind dashboard server to localhost by default (`run_dashboard.py`).
- [x] Add missing core error modules required by RW controller imports.
- [x] Restrict model file endpoints to project model directories only.

## Phase 2 - Backend Reliability

- [x] Fix path completion logic so endpoint position can complete path-following.
- [x] Harden multi-obstacle path generation clearance behavior.
- [x] Make preset speed tiers distinct (FAST/STABLE/PRECISION).
- [x] Align thruster-count tests with current 6-thruster model.
- [x] Validate with full backend test run (`423 passed, 15 skipped`).

## Phase 3 - Frontend Build Health (Pending)

- [x] Resolve TypeScript build errors (`npm run build` passes).
- [x] Resolve blocking ESLint errors (`npm run lint` passes with warnings).
- [x] Reconcile telemetry API types (`target_position`, `target_quaternion`).
- [x] Fix hook-rule violations that blocked lint.

## Phase 4 - Tooling and CI Alignment (Pending)

- [x] Add CI step for Python lint (`ruff`) with high-signal rules
      (`E9,F63,F7,F82,F401,F541,F841,E402,E722`).
- [x] Add CI job for frontend lint/build.
- [x] Reduce correctness lint debt in `src/` and `tests/` to zero
      for high-signal checks (unused imports/vars, import order, bare except, syntax class).
- [ ] Remaining style debt: `E501` long lines (`305` findings).
- [x] Add smoke tests for critical imports/modules.

## Phase 5 - Docs and DX (Pending)

- [x] Update docs to match current structure/commands.
- [x] Document local-only security model and localhost-only expectation.
- [x] Add a concise "known good local run" section with exact commands.
