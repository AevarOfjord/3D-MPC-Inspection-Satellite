# Third-Party Notices

This project depends on third-party software. The canonical dependency lists are:

- `pyproject.toml` (`[project.dependencies]` and `[project.optional-dependencies]`)
- `ui/package.json` (`dependencies` and `devDependencies`)

The project itself is licensed under MIT (`LICENSE`). Third-party components keep
their original licenses.

## Python Runtime Dependencies

- `aiofiles` (Apache-2.0)
- `fastapi` (MIT)
- `httpx` (BSD-3-Clause)
- `imageio` (BSD-2-Clause)
- `imageio-ffmpeg` (BSD-2-Clause)
- `matplotlib` (Matplotlib License / PSF-style)
- `numpy` (BSD-3-Clause)
- `osqp` (Apache-2.0)
- `pandas` (BSD-3-Clause)
- `plotly` (MIT)
- `psutil` (BSD-3-Clause)
- `pydantic` (MIT)
- `python-multipart` (Apache-2.0)
- `PyYAML` (MIT)
- `questionary` (MIT)
- `rich` (MIT)
- `scipy` (BSD-3-Clause)
- `seaborn` (BSD-3-Clause)
- `shapely` (BSD-3-Clause)
- `tqdm` (MPL-2.0 / MIT)
- `typer` (MIT)
- `uvicorn` (BSD-3-Clause)
- `watchfiles` (MIT)

## Python Development/Docs Dependencies

- `black` (MIT)
- `hypothesis` (MPL-2.0)
- `pre-commit` (MIT)
- `pytest-benchmark` (BSD-2-Clause)
- `pytest-cov` (MIT)
- `ruff` (MIT)
- `myst-parser` (MIT)
- `sphinx` (BSD-2-Clause)
- `sphinx-rtd-theme` (MIT)

## JavaScript Dependencies

JavaScript and frontend tooling licenses are governed by the dependencies in:

- `ui/package.json`
- `ui/package-lock.json`

Use this command to inspect exact resolved license entries locally:

```bash
cd ui
npx license-checker --summary
```

## How To Refresh This File

After dependency changes:

1. Update `pyproject.toml` and/or `ui/package.json`.
2. Re-run project checks (`make lint`, `make test-cov`, `cd ui && npm run build`).
3. Update this file if dependency names or major license families changed.
