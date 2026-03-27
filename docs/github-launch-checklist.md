# GitHub Launch Checklist

Use this checklist before publishing a public-facing branch or release of the project.

## Repo Hygiene

- `git status --short` is clean
- internal/dev-only files are not tracked
- runtime outputs are ignored and not staged
- generated caches and local build products are not staged
- README screenshots still match the current UI

## Public Narrative

- README opener describes the project as 3D inspection-satellite software
- README clearly states that this is simulation-first and not flight software
- showcase media highlights Mission Studio and playback/telemetry
- sample missions referenced in the README still exist and load

## Reproducibility

- `make install` works from a clean clone with the documented prerequisites
- `make ui-build` completes
- `make run-app` starts successfully
- acados prerequisites are documented accurately

## Quality Checks

- `make lint`
- `make test`
- `npm --prefix ui run test`
- `make docs-check`

## GitHub Surface

- repo description and topics match the public story
- issue templates and PR template are present
- `LICENSE`, `CONTRIBUTING.md`, `SECURITY.md`, and `CODE_OF_CONDUCT.md` are current
- any launch post or demo link points to the current default branch
