# Asset Attribution

This repository includes 3D assets under:

- `assets/model_files/` (canonical source)
- `ui/dist/model_files/` (build-time mirror produced by `make ui-build`)
- `ui/public/model_files/` (legacy local-dev copy; not canonical for releases)

Before distributing binaries or hosting this project publicly, confirm license
rights for each model and add source links and license text references below.

## Current Asset Inventory

| Asset Group | Canonical Path | Build Mirror Path | Source | License | Notes |
|---|---|---|---|---|---|
| Earth model | `assets/model_files/Earth/Earth.glb` | `ui/dist/model_files/Earth/Earth.glb` | TBD | TBD | Required for viewer/orbit snapshots |
| ISS model (GLB) | `assets/model_files/ISS/ISS.glb` | `ui/dist/model_files/ISS/ISS.glb` | TBD | TBD | Main orbit target |
| ISS model (OBJ/MTL) | `assets/model_files/ISS/ISS.obj`, `assets/model_files/ISS/ISS.mtl` | N/A | TBD | TBD | Used by scan tooling |
| Starlink model (GLB) | `assets/model_files/Starlink/starlink.glb` | `ui/dist/model_files/Starlink/starlink.glb` | TBD | TBD | Main orbit target |
| Starlink model (OBJ/MTL) | `assets/model_files/Starlink/starlink.obj`, `assets/model_files/Starlink/starlink.mtl` | N/A | TBD | TBD | Used by scan tooling |
| Sun model | `assets/model_files/Sun/Sun.glb` | `ui/dist/model_files/Sun/Sun.glb` | TBD | TBD | Solar system view |
| Sun USDZ | `assets/model_files/Sun/Sun_1_1391000.usdz` | N/A | TBD | TBD | Optional |
| Moon model | `assets/model_files/Moon/Moon.glb` | `ui/dist/model_files/Moon/Moon.glb` | TBD | TBD | Solar system view |
| Planet models | `assets/model_files/Planets/*.glb` | `ui/dist/model_files/Planets/*.glb` | TBD | TBD | Mercury, Venus, Mars, Jupiter, Saturn, Uranus, Neptune |
| User uploads | `assets/model_files/uploads/*` | N/A | User-provided | User-provided | Subject to uploader terms |

## Policy

- Keep `assets/model_files/` as the canonical source tree.
- Treat `ui/dist/model_files/` as the synchronized build mirror (`make sync-ui-model-assets`).
- Keep `ui/public/model_files/` as legacy dev-only assets and avoid using it as a release source of truth.
- Do not publish a release archive until every `TBD` entry is filled.
