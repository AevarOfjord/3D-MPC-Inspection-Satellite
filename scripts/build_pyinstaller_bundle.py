#!/usr/bin/env python3
"""Build cross-platform PyInstaller bundle and archive."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tarfile
import zipfile
from datetime import UTC, datetime
from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _platform_tag() -> str:
    if sys.platform.startswith("darwin"):
        return "macos"
    if sys.platform.startswith("win"):
        return "windows"
    return "linux"


def _data_sep() -> str:
    return ";" if sys.platform.startswith("win") else ":"


def _write_launcher(app_dir: Path) -> None:
    exe_name = "SatelliteControl.exe" if sys.platform.startswith("win") else "SatelliteControl"
    if sys.platform.startswith("win"):
        launcher = app_dir / "RUN_APP.bat"
        launcher.write_text(
            "@echo off\r\n"
            "setlocal\r\n"
            "set ROOT=%~dp0\r\n"
            "\"%ROOT%SatelliteControl.exe\" %*\r\n",
            encoding="utf-8",
        )
        return
    launcher = app_dir / "RUN_APP.sh"
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "ROOT_DIR=\"$(cd \"$(dirname \"$0\")\" && pwd)\"\n"
        f"exec \"$ROOT_DIR/{exe_name}\" \"$@\"\n",
        encoding="utf-8",
    )
    launcher.chmod(0o755)


def _archive_bundle(app_dir: Path, release_dir: Path, platform: str) -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%d_%H%M%S")
    if platform == "linux":
        archive = release_dir / f"SatelliteControl-{platform}-{stamp}.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            tar.add(app_dir, arcname="SatelliteControl")
        return archive

    archive = release_dir / f"SatelliteControl-{platform}-{stamp}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for item in app_dir.rglob("*"):
            if item.is_file():
                rel = item.relative_to(app_dir)
                zf.write(item, Path("SatelliteControl") / rel)
    return archive


def _assert_size_budget(archive: Path, max_mb: int) -> None:
    if max_mb <= 0:
        return
    size = archive.stat().st_size
    limit = max_mb * 1024 * 1024
    if size > limit:
        size_mb = (size + 1048575) // 1048576
        raise RuntimeError(
            f"Archive exceeds size budget: {archive} is {size_mb}MB (limit {max_mb}MB)."
        )


def _build_pyinstaller(root: Path, dist_dir: Path, work_dir: Path) -> Path:
    sep = _data_sep()
    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--onedir",
        "--name",
        "SatelliteControl",
        "--paths",
        str(root / "src" / "python"),
        "--distpath",
        str(dist_dir),
        "--workpath",
        str(work_dir / "work"),
        "--specpath",
        str(work_dir / "spec"),
    ]

    data_sources = [
        (root / "ui" / "dist", "ui/dist"),
        (root / "assets" / "model_files", "assets/model_files"),
        (root / "assets" / "scan_projects", "assets/scan_projects"),
        (root / "Data", "Data"),
    ]
    for source, target in data_sources:
        if source.exists():
            cmd.extend(["--add-data", f"{source}{sep}{target}"])

    entry = root / "scripts" / "packaged_entrypoint.py"
    cmd.append(str(entry))
    subprocess.check_call(cmd, cwd=str(root))
    bundle_dir = dist_dir / "SatelliteControl"
    if not bundle_dir.exists():
        raise RuntimeError(f"PyInstaller build did not create bundle: {bundle_dir}")
    return bundle_dir


def main() -> int:
    parser = argparse.ArgumentParser(description="Build Mission Control PyInstaller artifact.")
    parser.add_argument("--max-mb", type=int, default=150)
    args = parser.parse_args()

    root = _project_root()
    ui_dist = root / "ui" / "dist" / "index.html"
    if not ui_dist.exists():
        raise SystemExit("Prebuilt UI missing at ui/dist/index.html. Run `make ui-build` first.")

    platform = _platform_tag()
    release_dir = root / "release"
    dist_dir = release_dir / "pyinstaller" / platform
    work_dir = root / "build" / "pyinstaller" / platform
    if dist_dir.exists():
        shutil.rmtree(dist_dir)
    dist_dir.mkdir(parents=True, exist_ok=True)
    work_dir.mkdir(parents=True, exist_ok=True)
    release_dir.mkdir(parents=True, exist_ok=True)

    bundle_dir = _build_pyinstaller(root, dist_dir, work_dir)
    _write_launcher(bundle_dir)
    archive = _archive_bundle(bundle_dir, release_dir, platform)
    _assert_size_budget(archive, int(args.max_mb))

    size_mb = (archive.stat().st_size + 1048575) // 1048576
    print(f"Built PyInstaller bundle at {bundle_dir}")
    print(f"Created artifact: {archive} ({size_mb}MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
