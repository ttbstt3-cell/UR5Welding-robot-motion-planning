#!/usr/bin/env python3
"""Local-only, explicit-path Git checkpoints and verified immutable data snapshots.

No ROS/MATLAB imports, network commands, automatic scheduling, deletion of old
snapshots, or restoration over existing files. See maintenance/BACKUP_WORKFLOW.md.
"""
import argparse
import collections
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parents[1]
CONTEXT = ("AGENTS.md", "PROJECT_STATE.md", "HANDOFF.md")
SECRET_PATTERNS = (
    ("private-key", re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----")),
    ("github-token", re.compile(rb"\bgh[pousr]_[A-Za-z0-9]{25,}\b")),
    ("github-pat", re.compile(rb"\bgithub_pat_[A-Za-z0-9_]{40,}\b")),
    ("api-key", re.compile(rb"\bsk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{32,}\b")),
    ("aws-access-key", re.compile(rb"\b(?:AKIA|ASIA)[A-Z0-9]{16}\b")),
    ("credential-url", re.compile(rb"https?://[^\s/@:]+:[^\s/@]+@")),
)


def stamp():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Only our mutable status/index files use replacement, never source payloads.
    fd, tmp = tempfile.mkstemp(prefix=".maintenance-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as out:
            json.dump(data, out, ensure_ascii=False, indent=2)
            out.write("\n")
            out.flush()
            os.fsync(out.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as src:
        for block in iter(lambda: src.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def git(root, *args, check=True):
    result = subprocess.run(["git", "-C", str(root), *args],
                            capture_output=True, text=True)
    if check and result.returncode:
        raise RuntimeError("git " + args[0] + " failed: " + result.stderr.strip())
    return result.stdout.strip() if check else result


def safe_relative(value):
    p = PurePosixPath(value)
    if (p.is_absolute() or not p.parts or any(x in ("..", ".") for x in p.parts)
            or any(c in value for c in ("\n", "\r", "\0"))):
        raise ValueError("Unsafe relative path")
    return p.as_posix()


def below(path, prefix):
    return path == prefix or path.startswith(prefix + "/")


def secret_name(name):
    return (name in (".git-credentials", ".netrc", "id_rsa", "id_ed25519")
            or (name.startswith(".env") and name not in (".env.example", ".env.template"))
            or Path(name).suffix.lower() in (".pem", ".key", ".p12", ".pfx"))


def fingerprint(info):
    return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns,
            info.st_ctime_ns, stat.S_IMODE(info.st_mode)]


class Maintenance:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.policy = read_json(self.root / "maintenance/backup_policy.json")
        self.store = self.root / safe_relative(self.policy["backup_directory"])

    def walk(self):
        """Metadata only; never follows links or enters excluded directories."""
        excluded = set(self.policy["exclude_directory_names"])
        prefixes = self.policy["exclude_path_prefixes"]
        for folder, dirs, files in os.walk(self.root, followlinks=False):
            base = Path(folder)
            dirs[:] = sorted(d for d in dirs if d not in excluded
                             and not any(below((base / d).relative_to(self.root).as_posix(), p)
                                         for p in prefixes))
            links = [d for d in dirs if (base / d).is_symlink()]
            dirs[:] = [d for d in dirs if d not in links]
            for name in sorted(files + links):
                if secret_name(name) or name in excluded:
                    continue
                path = base / name
                rel = path.relative_to(self.root).as_posix()
                info = path.lstat()
                if stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
                    yield rel, info

    def current_runs(self):
        prefixes = {"UR5/规划文件/当前轨迹"}
        index = self.root / "UR5/规划文件/当前轨迹/最新轨迹索引.json"
        if index.exists():
            def visit(obj):
                if isinstance(obj, dict):
                    for key, val in obj.items():
                        if key == "source_run" and isinstance(val, str):
                            path = Path(val)
                            if path.is_absolute() and path.is_relative_to(self.root):
                                prefixes.add(path.relative_to(self.root).as_posix())
                        visit(val)
                elif isinstance(obj, list):
                    for val in obj:
                        visit(val)
            visit(read_json(index))
        return prefixes

    def git_candidate(self, rel, info, runs):
        if stat.S_ISLNK(info.st_mode):
            return False  # Link targets are explicitly recorded in snapshots.
        if info.st_size > self.policy["git_max_file_bytes"]:
            return False
        if any(below(rel, p) for p in self.policy["git_archive_only_prefixes"]):
            return False
        suffix = Path(rel).suffix.lower()
        return (suffix in self.policy["git_text_suffixes"]
                or suffix in self.policy["git_mesh_suffixes"]
                or rel in self.policy["git_mat_paths"]
                or any(below(rel, p) for p in runs | set(self.policy["git_evidence_prefixes"]))
                or Path(rel).name in (".gitignore", "CMakeLists.txt", "Makefile", "LICENSE"))

    def inventory(self, write=False):
        files = list(self.walk())
        runs = self.current_runs()
        candidates = sorted(rel for rel, info in files if self.git_candidate(rel, info, runs))
        ignore_check = subprocess.run(["git", "-C", str(self.root), "check-ignore", "--stdin", "-z"],
                                      input=("\0".join(candidates) + "\0").encode(), capture_output=True)
        if ignore_check.returncode not in (0, 1):
            raise RuntimeError("Cannot inspect ignore rules for explicit candidate list")
        ignored = set(ignore_check.stdout.decode().split("\0")) - {""}
        candidates = [p for p in candidates if p not in ignored]
        by_root = collections.Counter()
        by_ext = collections.Counter()
        large = []
        for rel, info in files:
            by_root[rel.split("/")[0]] += info.st_size
            by_ext[Path(rel).suffix.lower()] += info.st_size
            if info.st_size > self.policy["git_max_file_bytes"]:
                large.append({"path": rel, "bytes": info.st_size})
        result = {"created_at": stamp(), "files": len(files),
                  "bytes": sum(i.st_size for _, i in files),
                  "git_candidates": len(candidates), "bytes_by_root": dict(by_root),
                  "git_ignored_candidates": sorted(ignored),
                  "bytes_by_extension": dict(by_ext), "large_files": large,
                  "current_run_prefixes": sorted(runs),
                  "scope": "Project only; policy exclusions, no symlink traversal."}
        if write:
            write_json(self.root / "maintenance/inventory.json", result)
            # An explicit, inspectable list; never a wildcard git add.
            candidates = sorted(set(candidates) | {"maintenance/inventory.json", "maintenance/git_paths.txt"})
            (self.root / "maintenance/git_paths.txt").write_text(
                "\n".join(candidates) + "\n", encoding="utf-8")
        return result

    def scan(self, paths=None):
        findings, skipped, scanned = [], [], 0
        paths = paths if paths is not None else [p for p, _ in self.walk()]
        text = set(self.policy["git_text_suffixes"])
        for rel in paths:
            path = self.root / safe_relative(rel)
            if secret_name(path.name):
                findings.append({"path": rel, "kind": "sensitive-filename"})
                continue
            if path.is_symlink() or not path.is_file():
                continue
            if path.suffix.lower() not in text and path.name != ".gitignore":
                continue
            if path.stat().st_size > 16 * 1024 * 1024:
                skipped.append(rel)
                continue
            raw = path.read_bytes()
            scanned += 1
            for kind, pattern in SECRET_PATTERNS:
                for hit in pattern.finditer(raw):
                    findings.append({"path": rel, "line": raw[:hit.start()].count(b"\n") + 1,
                                     "kind": kind})
        return {"scanned_text_files": scanned, "findings": findings,
                "large_text_not_scanned": skipped,
                "limitations": "High-confidence patterns only; binary assets are not content-scanned. No cloud upload."}

    def verify_context(self):
        sizes = {name: (self.root / name).stat().st_size for name in CONTEXT}
        if sum(sizes.values()) > self.policy["context_total_max_bytes"]:
            raise RuntimeError("Context entry documents exceed configured budget")
        for name in CONTEXT:
            value = (self.root / name).read_text(encoding="utf-8")
            if not value.startswith("# ") or "\n## " not in value:
                raise RuntimeError("Context headings missing: " + name)
        return {"status": "PASS", "bytes": sizes, "total_bytes": sum(sizes.values())}

    def latest(self):
        return read_json(self.store / "latest.json") if (self.store / "latest.json").exists() else None

    def status(self):
        head = git(self.root, "rev-parse", "--verify", "HEAD", check=False)
        untracked = git(self.root, "ls-files", "--others", "--exclude-standard", "-z")
        return {"branch": git(self.root, "branch", "--show-current"),
                "head": head.stdout.strip() if head.returncode == 0 else None,
                "tracked_changes": git(self.root, "status", "--porcelain=v1", "--untracked-files=no"),
                "untracked_file_count": len([x for x in untracked.split("\0") if x]),
                "remote_names": git(self.root, "remote").splitlines(),
                "last_verified_snapshot": self.latest(),
                "remote_sync_performed_by_this_command": False}

    def snapshot_dir(self, snapshot=None):
        snapshot = snapshot or (self.latest() or {}).get("snapshot")
        if not snapshot or Path(snapshot).name != snapshot or snapshot in (".", ".."):
            raise ValueError("Valid snapshot ID required")
        return self.store / "snapshots" / snapshot

    def verify(self, snapshot=None):
        folder = self.snapshot_dir(snapshot)
        manifest_path = folder / "manifest.json"
        manifest = read_json(manifest_path)
        if (folder / "verified.json").exists():
            if read_json(folder / "verified.json")["manifest_sha256"] != sha256(manifest_path):
                raise RuntimeError("Manifest checksum mismatch")
        checked = set()
        for item in manifest["files"]:
            if item["type"] != "file" or item["sha256"] in checked:
                continue
            digest = item["sha256"]
            obj = self.store / "objects" / digest[:2] / digest
            if obj.stat().st_size != item["bytes"] or sha256(obj) != digest:
                raise RuntimeError("Snapshot payload mismatch: " + item["path"])
            checked.add(digest)
        for bundle in manifest["git_bundles"]:
            file = folder / bundle["file"]
            if sha256(file) != bundle["sha256"]:
                raise RuntimeError("Git bundle checksum mismatch")
            git(self.root, "bundle", "verify", str(file))
        return {"status": "PASS", "snapshot": folder.name, "files": len(manifest["files"]),
                "unique_objects_checked": len(checked), "git_bundles_checked": len(manifest["git_bundles"]),
                "manifest_sha256": sha256(manifest_path), "verified_at": stamp()}

    def backup(self):
        self.store.mkdir(mode=0o700, exist_ok=True)
        with (self.store / "operation.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            scan = self.scan()
            if scan["findings"]:
                raise RuntimeError("Sensitive-content scan blocked backup: " + json.dumps(scan["findings"], ensure_ascii=False))
            snapshot = dt.datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
            folder = self.store / "snapshots" / snapshot
            folder.mkdir(parents=True, mode=0o700)
            old = {}
            if self.latest():
                old = {v["path"]: v for v in read_json(self.snapshot_dir() / "manifest.json")["files"]}
            files, identities, copied, reused = [], {}, 0, 0
            for rel, info in self.walk():
                path = self.root / rel
                identities[rel] = fingerprint(info)
                if stat.S_ISLNK(info.st_mode):
                    files.append({"path": rel, "type": "symlink", "target": os.readlink(path),
                                  "source_identity": fingerprint(info)})
                    continue
                previous = old.get(rel, {})
                if previous.get("source_identity") == fingerprint(info) and previous.get("sha256"):
                    digest = previous["sha256"]
                    obj = self.store / "objects" / digest[:2] / digest
                    if not obj.is_file() or obj.stat().st_size != info.st_size:
                        raise RuntimeError("Previous backup object missing; refusing silent reuse")
                    reused += 1
                else:
                    temp_fd, temporary = tempfile.mkstemp(prefix="payload-", dir=folder)
                    try:
                        h = hashlib.sha256()
                        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
                        with os.fdopen(fd, "rb") as src, os.fdopen(temp_fd, "wb") as dst:
                            if fingerprint(os.fstat(src.fileno())) != fingerprint(info):
                                raise RuntimeError("Source changed before read: " + rel)
                            for block in iter(lambda: src.read(4 * 1024 * 1024), b""):
                                h.update(block)
                                dst.write(block)
                            dst.flush()
                            os.fsync(dst.fileno())
                            if fingerprint(os.fstat(src.fileno())) != fingerprint(info):
                                raise RuntimeError("Source changed during read: " + rel)
                        digest = h.hexdigest()
                        obj = self.store / "objects" / digest[:2] / digest
                        obj.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                        os.chmod(temporary, 0o400)
                        try:
                            os.link(temporary, obj)  # No overwrite; never linked to live source.
                            copied += info.st_size
                        except FileExistsError:
                            pass  # Full verification below checks existing bytes.
                    finally:
                        os.unlink(temporary)  # Only our newly-created temporary file.
                files.append({"path": rel, "type": "file", "bytes": info.st_size,
                              "mode": stat.S_IMODE(info.st_mode), "sha256": digest,
                              "source_identity": fingerprint(info)})
            bundles = []
            for number, rel in enumerate(["."] + self.policy["nested_repositories"]):
                repo = self.root / rel
                head = git(repo, "rev-parse", "--verify", "HEAD", check=False)
                if head.returncode:
                    raise RuntimeError("No committed history for " + rel)
                file = "root.bundle" if number == 0 else f"nested-{number}.bundle"
                git(repo, "bundle", "create", str(folder / file), "--all")
                git(repo, "bundle", "verify", str(folder / file))
                if git(repo, "rev-parse", "HEAD") != head.stdout.strip():
                    raise RuntimeError("Git HEAD changed during backup")
                bundles.append({"repository": rel, "head": head.stdout.strip(),
                                "file": file, "sha256": sha256(folder / file)})
            after = {rel: fingerprint(info) for rel, info in self.walk()}
            if identities != after:
                raise RuntimeError("Project changed during snapshot; no verified/latest snapshot published")
            manifest = {"schema_version": 1, "snapshot": snapshot, "created_at": stamp(),
                        "project_root": str(self.root), "files": files, "git_bundles": bundles,
                        "policy": self.policy, "secret_scan": scan,
                        "physical_location": "Same project disk; not offsite or cloud", "new_object_bytes": copied,
                        "unchanged_file_reuses": reused, "execution_state": "No robot/ROS actions"}
            write_json(folder / "manifest.json", manifest)
            receipt = self.verify(snapshot)
            write_json(folder / "verified.json", receipt)
            write_json(self.store / "latest.json", receipt)
            return {**receipt, "new_object_bytes": copied, "unchanged_file_reuses": reused}

    def restore(self, destination, paths, snapshot=None):
        folder = self.snapshot_dir(snapshot)
        dest = Path(destination).absolute()
        resolved = dest.resolve()
        if (dest.exists() or dest.is_symlink() or resolved == self.root
                or resolved in self.root.parents or self.root in resolved.parents):
            raise ValueError("Restore requires a NEW isolated destination outside active project")
        manifest = read_json(folder / "manifest.json")
        receipt = read_json(folder / "verified.json")
        if receipt["manifest_sha256"] != sha256(folder / "manifest.json"):
            raise RuntimeError("Manifest checksum mismatch")
        requested = set(safe_relative(p) for p in paths)
        selected = [v for v in manifest["files"] if v["path"] in requested]
        if len(selected) != len(requested):
            raise ValueError("Requested path missing from snapshot")
        if any(v["type"] != "file" for v in selected):
            raise ValueError("Symlink restore requires manual target review; nothing restored")
        for item in selected:
            obj = self.store / "objects" / item["sha256"][:2] / item["sha256"]
            if sha256(obj) != item["sha256"]:
                raise RuntimeError("Restore payload mismatch")
        dest.mkdir(parents=True, exist_ok=False)
        for item in selected:
            target = dest / safe_relative(item["path"])
            target.parent.mkdir(parents=True, exist_ok=True)
            obj = self.store / "objects" / item["sha256"][:2] / item["sha256"]
            shutil.copyfile(obj, target)  # Never hardlink immutable backup into editable restore.
            os.chmod(target, item["mode"])
            if sha256(target) != item["sha256"]:
                raise RuntimeError("Restored file mismatch")
        return {"status": "PASS", "snapshot": folder.name, "restored_files": len(selected),
                "destination": str(dest), "active_project_overwritten": False}

    def catalog(self):
        manifest = read_json(self.snapshot_dir() / "manifest.json")
        tracked = set(git(self.root, "ls-files", "-z").split("\0"))
        assets = [{k: item[k] for k in ("path", "bytes", "sha256")}
                  for item in manifest["files"] if item["type"] == "file" and item["path"] not in tracked]
        result = {"snapshot": manifest["snapshot"], "assets": assets,
                  "note": "Payloads are local immutable snapshot objects, NOT Git objects or a remote backup."}
        write_json(self.root / "maintenance/asset_catalog.json", result)
        return {"catalog_assets": len(assets), "bytes": sum(v["bytes"] for v in assets)}

    def checkpoint(self, paths, message, tests):
        self.verify_context()
        initial_baseline = git(self.root, "rev-parse", "--verify", "HEAD", check=False).returncode != 0
        if git(self.root, "diff", "--cached", "--name-only", "-z"):
            raise RuntimeError("Pre-existing staged changes: stop without altering index")
        paths = sorted(set(safe_relative(p) for p in paths))
        if "HANDOFF.md" not in paths or not tests.strip():
            raise ValueError("Explicit HANDOFF.md and actual test description required")
        for rel in paths:
            path = self.root / rel
            if path.is_symlink() or not path.is_file() or self.root not in path.resolve().parents:
                raise ValueError("Only explicit in-project regular files: " + rel)
        scan = self.scan(paths)
        if scan["findings"]:
            raise RuntimeError("Sensitive-content scan blocked checkpoint: " + json.dumps(scan["findings"], ensure_ascii=False))
        # NUL separated literal pathspecs preserve Chinese names and prevent wildcard expansion.
        payload = ("\0".join(paths) + "\0").encode("utf-8")
        result = subprocess.run(["git", "--literal-pathspecs", "-C", str(self.root), "add",
                                 "--pathspec-from-file=-", "--pathspec-file-nul"],
                                input=payload, capture_output=True)
        if result.returncode:
            raise RuntimeError("Explicit staging failed; inspect index (not auto-reset)")
        whitespace = git(self.root, "diff", "--cached", "--check", check=False)
        if whitespace.returncode and not initial_baseline:
            raise RuntimeError("Staged whitespace check failed; inspect git diff --cached --check (index preserved)")
        if whitespace.returncode:
            print(json.dumps({"baseline_note": "Existing whitespace preserved, not reformatted",
                              "diagnostic_lines": len(whitespace.stdout.splitlines())}), flush=True)
        identity = []
        if not git(self.root, "config", "user.name", check=False).stdout.strip():
            identity += ["-c", "user.name=Local Project Maintenance"]
        if not git(self.root, "config", "user.email", check=False).stdout.strip():
            identity += ["-c", "user.email=local-maintenance@localhost"]
        git(self.root, *identity, "commit", "-m", message, "-m", "Validation: " + tests)
        head = git(self.root, "rev-parse", "HEAD")
        print(json.dumps({"committed_head": head, "next": "local payload snapshot and verification"}), flush=True)
        # A failure here preserves the commit but does NOT claim the backup succeeded.
        return {"head": head, "backup": self.backup()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("status", "verify-context", "scan", "backup", "catalog"):
        sub.add_parser(command)
    sub.add_parser("inventory").add_argument("--write", action="store_true")
    sub.add_parser("verify").add_argument("--snapshot")
    restore = sub.add_parser("restore")
    restore.add_argument("--snapshot")
    restore.add_argument("--destination", required=True)
    restore.add_argument("--paths", nargs="+", required=True)
    cp = sub.add_parser("checkpoint")
    cp.add_argument("--message", required=True)
    cp.add_argument("--tests", required=True)
    group = cp.add_mutually_exclusive_group(required=True)
    group.add_argument("--paths", nargs="+")
    group.add_argument("--paths-file")
    args = parser.parse_args()
    maintenance = Maintenance(ROOT)
    if args.command == "verify-context":
        result = maintenance.verify_context()
    elif args.command == "checkpoint":
        paths = args.paths or (ROOT / safe_relative(args.paths_file)).read_text(encoding="utf-8").splitlines()
        result = maintenance.checkpoint(paths, args.message, args.tests)
    elif args.command == "restore":
        result = maintenance.restore(args.destination, args.paths, args.snapshot)
    elif args.command == "inventory":
        result = maintenance.inventory(args.write)
        result = {key: val for key, val in result.items() if key not in ("large_files", "bytes_by_root", "bytes_by_extension")}
    elif args.command == "verify":
        result = maintenance.verify(args.snapshot)
    else:
        result = getattr(maintenance, args.command)()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.command == "scan" and result["findings"]:
        return 2
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, ValueError, OSError) as exc:
        print("MAINTENANCE_FAILED: " + str(exc), file=sys.stderr)
        sys.exit(1)
