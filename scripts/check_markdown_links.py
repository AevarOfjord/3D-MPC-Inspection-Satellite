#!/usr/bin/env python3
"""Validate markdown link targets and repository path references."""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

MARKDOWN_LINK_RE = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
INLINE_CODE_RE = re.compile(r"`([^`]+)`")
PATH_TOKEN_RE = re.compile(
    r"(?<![\w./-])(?P<path>(?:\./)?(?:src|ui|tests|docs|scripts|missions|data)/[A-Za-z0-9._/\-]+/?)(?![\w./-])"
)


def _iter_markdown_files() -> list[Path]:
    root_files = sorted(p for p in REPO_ROOT.glob("*.md") if p.is_file())
    docs_files = sorted(p for p in (REPO_ROOT / "docs").rglob("*.md"))
    return root_files + docs_files


def _resolve_relative(base_file: Path, raw_target: str) -> Path:
    return (base_file.parent / raw_target).resolve()


def _path_candidates(raw: str) -> list[Path]:
    token = raw.strip().rstrip(".,:;)")
    rel = token[2:] if token.startswith("./") else token

    candidates = [REPO_ROOT / rel]
    if rel.startswith("src/"):
        candidates.append(REPO_ROOT / "ui" / rel)
    if rel.startswith("tests/"):
        candidates.append(REPO_ROOT / "ui" / rel)
    return candidates


def _check_markdown_links(md_file: Path) -> list[str]:
    failures: list[str] = []
    text = md_file.read_text(encoding="utf-8")
    for match in MARKDOWN_LINK_RE.finditer(text):
        raw_target = match.group(1).strip()
        if not raw_target:
            continue
        if raw_target.startswith(("http://", "https://", "mailto:", "#")):
            continue
        target = raw_target.split("#", 1)[0].strip()
        if not target:
            continue
        resolved = _resolve_relative(md_file, target)
        if not resolved.exists():
            failures.append(
                f"{md_file.relative_to(REPO_ROOT)}: missing markdown link target '{raw_target}'"
            )
    return failures


def _check_inline_path_references(md_file: Path) -> list[str]:
    failures: list[str] = []
    text = md_file.read_text(encoding="utf-8")
    for code_match in INLINE_CODE_RE.finditer(text):
        snippet = code_match.group(1)
        for path_match in PATH_TOKEN_RE.finditer(snippet):
            token = path_match.group("path")
            candidates = _path_candidates(token)
            if not any(candidate.exists() for candidate in candidates):
                failures.append(
                    f"{md_file.relative_to(REPO_ROOT)}: missing repo path reference '{token}'"
                )
    return failures


def main() -> int:
    failures: list[str] = []
    for md_file in _iter_markdown_files():
        failures.extend(_check_markdown_links(md_file))
        failures.extend(_check_inline_path_references(md_file))

    if failures:
        for item in sorted(set(failures)):
            print(item)
        print(f"\nmarkdown check failed: {len(set(failures))} issue(s)")
        return 1

    print("markdown check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
