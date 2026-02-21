# Contributing

## Development Setup

1. Install prerequisites (Python 3.11, Node.js, CMake/Ninja).
2. Run:

```bash
make install
```

3. Build frontend when needed:

```bash
make ui-build
```

## Quality Gates

Before opening a PR, run:

```bash
make lint
make test
npm --prefix ui run test
```

## Pull Request Rules

- Keep changes focused.
- Include tests for behavior changes.
- Update docs when commands, paths, or APIs change.
- Avoid mixing refactors with feature work unless required.
