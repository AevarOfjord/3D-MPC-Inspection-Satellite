# V4 Release Checklist

This checklist closes the V4.0 rollout path (`beta.1 -> rc.1 -> final`) for local-first desktop release.

## 1) Pre-release gates

```bash
make lint
make test-cov
cd ui && npm run lint && npm run build && npm run test && npm run test:e2e
make docs-build
make package-app
make package-pyinstaller
```

## 2) V4 KPI validation

Run automated KPI + desktop/a11y readiness scenarios:

```bash
cd ui && npx playwright test tests/e2e/v4_readiness.spec.ts --config playwright.config.ts
```

Expected:
- mission creation + validation + save-ready scenario completes in under 5 minutes
- planner layout remains stable at 1280 / 1440 / 1920 widths
- keyboard flow supports focus-visible and step navigation

## 3) Beta tag and notes

After all gates pass and release notes are reviewed:

```bash
git add -A
git commit -m "release: v4.0.0-beta.1"
git tag -a v4.0.0-beta.1 -m "V4.0.0 beta.1"
git push origin main --follow-tags
```

## 4) Final tag

After beta feedback and RC hardening:

```bash
git add -A
git commit -m "release: v4.0.0"
git tag -a v4.0.0 -m "V4.0.0"
git push origin main --follow-tags
```

## 5) Packaging artifact expectations

- `make package-app`: archive under `release/` and within `PACKAGE_MAX_MB` budget
- `make package-pyinstaller`: OS-native bundle and archive under `release/pyinstaller/`
- packaged app launches at `http://127.0.0.1:8000`
