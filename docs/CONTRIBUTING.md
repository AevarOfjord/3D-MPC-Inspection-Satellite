# Contributing Guide

Thanks for contributing to Satellite Control.

## Development Setup

```bash
git clone https://github.com/AevarOfjord/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel.git
cd Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel
make install
make ui-build
```text
## Daily Commands

```bash
make run         # backend + frontend dev mode
make run-app     # backend serves prebuilt UI
make lint        # backend + frontend lint
make test-cov    # tests + coverage gate
make docs-build  # docs build with warnings as errors
```text
## Pull Request Checklist

1. Keep changes focused and include rationale in the PR description.
2. Run `make lint` and `make test-cov` locally.
3. Update docs when behavior, commands, or APIs change.
4. If dependencies/assets change, update:
   - `THIRD_PARTY_NOTICES.md`
   - `ASSET_ATTRIBUTION.md`
5. Add or update tests for bug fixes and new behavior.

## Branching and Commits

- Branch from `main`.
- Use clear commit messages that explain intent.
- Do not force-push over other contributors' work.

## Reporting Issues

- Bug reports: use the Bug Report issue template.
- Feature requests: use the Feature Request template.
- Security issues: follow `SECURITY.md` (do not file public security issues first).

## Code Style

- Python: `ruff`, `black`, typed where practical.
- TypeScript: `eslint`, `tsc`.
- Keep public behavior backward-compatible unless clearly documented.
