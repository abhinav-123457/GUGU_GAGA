"""Run-manifest recording: everything needed to say "this exact run used
this exact code, this exact config, and these exact random streams."

This repository has no `.git` directory today (confirmed: `Is a git
repository: false`), so `get_code_version()` cannot report a commit hash.
Rather than silently omitting code identity, it falls back to a content
hash of `swarm_sim/`'s own source files, so two runs against identical code
always report the same `code_version` and any source change - even
uncommitted - changes it. If this project is later put under git,
`get_code_version()` prefers `git rev-parse HEAD` (suffixed `+dirty` if the
working tree has uncommitted changes) automatically, with no caller change
needed.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import platform
import subprocess
from importlib import metadata as importlib_metadata
from pathlib import Path
from typing import Optional

from .contracts import CONTRACT_VERSION, check_contract_version_compatible

_PACKAGE_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _PACKAGE_DIR.parent

_TRACKED_DEPENDENCIES = ("numpy", "scipy", "matplotlib", "pybullet", "gymnasium", "transforms3d")


def get_code_version(repo_root: Optional[Path] = None, package_dir: Optional[Path] = None) -> str:
    repo_root = Path(repo_root) if repo_root is not None else _REPO_ROOT
    package_dir = Path(package_dir) if package_dir is not None else _PACKAGE_DIR

    if (repo_root / ".git").is_dir():
        try:
            commit = subprocess.run(
                ["git", "rev-parse", "HEAD"], cwd=repo_root,
                capture_output=True, text=True, check=True, timeout=5,
            ).stdout.strip()
            status = subprocess.run(
                ["git", "status", "--porcelain"], cwd=repo_root,
                capture_output=True, text=True, check=True, timeout=5,
            ).stdout
            dirty = "+dirty" if status.strip() else ""
            return f"git:{commit}{dirty}"
        except Exception:
            pass  # fall through to the source-hash fallback below

    hasher = hashlib.sha256()
    for path in sorted(package_dir.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        rel = path.relative_to(package_dir).as_posix()
        hasher.update(rel.encode("utf-8"))
        hasher.update(path.read_bytes())
    return f"srchash:{hasher.hexdigest()[:16]}"


def get_dependency_versions(names=_TRACKED_DEPENDENCIES) -> dict:
    versions = {}
    for name in names:
        try:
            versions[name] = importlib_metadata.version(name)
        except importlib_metadata.PackageNotFoundError:
            versions[name] = "not-installed"
    return versions


def hash_event_log(events) -> str:
    """Deterministic hash of a sequence of JSON-compatible event dicts.
    Ready for later phases' event loggers to call; Phase 1 itself produces
    no event log, so `event_log_hash` on a manifest built today is None."""
    canonical = json.dumps(list(events), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _config_to_dict(config) -> dict:
    if isinstance(config, dict):
        return dict(config)
    if dataclasses.is_dataclass(config):
        return dataclasses.asdict(config)
    raise ValueError(f"config must be a dict or dataclass instance, got {type(config)!r}")


@dataclasses.dataclass(frozen=True)
class RunManifest:
    contract_version: str
    config: dict
    master_seed: int
    subsystem_seeds: dict
    code_version: str
    python_version: str
    dependency_versions: dict
    map_id: str
    model_id: str
    event_log_hash: Optional[str] = None

    # Only this field may be legitimately absent from serialized data: it
    # is None whenever a run produced no event log (true of every Phase 1
    # manifest, since Phase 1 adds no event logger).
    _OPTIONAL_FIELDS = frozenset({"event_log_hash"})

    def __post_init__(self):
        if not isinstance(self.contract_version, str) or not self.contract_version:
            raise ValueError("contract_version must be a non-empty str")
        check_contract_version_compatible(self.contract_version, context="RunManifest")
        if not isinstance(self.config, dict):
            raise ValueError("config must be a dict")
        if not isinstance(self.master_seed, int) or isinstance(self.master_seed, bool):
            raise ValueError("master_seed must be an int")
        if not isinstance(self.subsystem_seeds, dict):
            raise ValueError("subsystem_seeds must be a dict")
        if not isinstance(self.code_version, str) or not self.code_version:
            raise ValueError("code_version must be a non-empty str")
        if not isinstance(self.map_id, str) or not self.map_id:
            raise ValueError("map_id must be a non-empty str")
        if not isinstance(self.model_id, str) or not self.model_id:
            raise ValueError("model_id must be a non-empty str")

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "RunManifest":
        if not isinstance(data, dict):
            raise ValueError("RunManifest.from_dict requires a dict")
        if "contract_version" not in data:
            raise ValueError("RunManifest.from_dict requires 'contract_version' in serialized data")
        check_contract_version_compatible(data["contract_version"], context="RunManifest")

        field_names = {f.name for f in dataclasses.fields(cls)}
        required = field_names - cls._OPTIONAL_FIELDS
        missing = sorted(n for n in required if n not in data)
        if missing:
            raise ValueError(f"RunManifest.from_dict missing required fields: {missing}")

        return cls(**{name: data[name] for name in field_names if name in data})

    def save_json(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True))

    @classmethod
    def load_json(cls, path) -> "RunManifest":
        return cls.from_dict(json.loads(Path(path).read_text()))


def build_run_manifest(config, master_seed, seed_manager, map_id="unspecified",
                        model_id="unspecified", event_log=None) -> RunManifest:
    """Assemble a RunManifest for one mission run. `seed_manager` should be
    the same SeedManager instance the run actually used, so
    `subsystem_seeds` reflects every subsystem stream that was really drawn
    from (only subsystems whose `.rng()`/`.seed_for()` was called by the
    time this is built will appear - call this after the run, not before)."""
    return RunManifest(
        contract_version=CONTRACT_VERSION,
        config=_config_to_dict(config),
        master_seed=master_seed,
        subsystem_seeds=seed_manager.derived_seeds(),
        code_version=get_code_version(),
        python_version=platform.python_version(),
        dependency_versions=get_dependency_versions(),
        map_id=map_id,
        model_id=model_id,
        event_log_hash=hash_event_log(event_log) if event_log is not None else None,
    )
