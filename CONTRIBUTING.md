# Contributing

Thanks for taking a look at the project.

This repository is published as a public research/demo codebase. Contributions that improve clarity, reproducibility, mission authoring, controller evaluation, and platform stability are welcome.

## Development Setup

1. Install the documented prerequisites:
   - Python 3.11
   - Node.js 20+
   - CMake + Ninja
2. Create or repair the local environment:

```bash
make install
```

3. Build the frontend when needed:

```bash
make ui-build
```

4. Run the packaged-style local app:

```bash
make run-app
```

For frontend development, use:

```bash
make run
```

## Quality Gates

Before opening a PR, run the checks that match your change:

```bash
make lint
make test
npm --prefix ui run test
```

If you change docs or repository-facing paths, also run:

```bash
make docs-check
```

## Contribution Guidelines

- Keep PRs focused; avoid mixing cleanup, refactors, and feature work unless they are tightly related.
- Add or update tests for behavior changes.
- Update docs when commands, workflows, screenshots, or public-facing behavior change.
- Do not commit runtime outputs, sweep artifacts, local caches, or generated build clutter.
- Prefer canonical controller/profile names and documented Make targets in user-facing docs.

## Good First Contribution Areas

- Mission Studio UX polish
- playback / telemetry inspection improvements
- controller comparison and sweep reporting
- installation and reproducibility fixes
- documentation accuracy and public-facing examples

## Security

Please do not open public issues for potential security problems. Follow the process in [SECURITY.md](SECURITY.md).
