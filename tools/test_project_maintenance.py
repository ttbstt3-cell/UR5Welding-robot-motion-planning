"""Isolated fixtures only: no real project writes, robot imports or ROS startup."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from project_maintenance import Maintenance, ROOT, safe_relative, sha256


class MaintenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ur5-maintenance-test-")
        self.root = Path(self.temp.name) / "project"
        self.root.mkdir()
        (self.root / "maintenance").mkdir()
        policy = json.loads((ROOT / "maintenance/backup_policy.json").read_text())
        policy["nested_repositories"] = []
        (self.root / "maintenance/backup_policy.json").write_text(json.dumps(policy))
        for name in ("AGENTS.md", "PROJECT_STATE.md", "HANDOFF.md"):
            (self.root / name).write_text("# Fixture\n\n## State\nOffline test only.\n")
        (self.root / "source.py").write_text("value = 1\n")
        (self.root / "result.mat").write_bytes(b"fixture-binary-data\0\1\2")
        self.git("init", "--initial-branch=main")
        self.git("add", "AGENTS.md", "PROJECT_STATE.md", "HANDOFF.md", "source.py")
        self.git("-c", "user.name=Fixture", "-c", "user.email=fixture@localhost",
                 "commit", "-m", "Fixture baseline")
        self.m = Maintenance(self.root)

    def tearDown(self):
        self.temp.cleanup()  # Only this test's private temporary fixture.

    def git(self, *args):
        return subprocess.run(["git", "-C", str(self.root), *args],
                              check=True, capture_output=True, text=True)

    def test_context_budget(self):
        self.assertEqual(self.m.verify_context()["status"], "PASS")
        (self.root / "HANDOFF.md").write_text("# State\n## More\n" + "x" * 17000)
        with self.assertRaises(RuntimeError):
            self.m.verify_context()

    def test_no_secret_values_in_findings(self):
        token = "ghp_" + "A" * 40
        (self.root / "source.py").write_text("token = '" + token + "'\n")
        scan = self.m.scan()
        self.assertEqual(scan["findings"][0]["kind"], "github-token")
        self.assertNotIn(token, json.dumps(scan))
        with self.assertRaises(RuntimeError):
            self.m.backup()
        self.assertIsNone(self.m.latest())

    def test_secrets_and_builds_excluded_research_preserved(self):
        (self.root / ".env").write_text("secret")
        (self.root / ".env.example").write_text("VALUE=example")
        (self.root / "ros2_ws/build").mkdir(parents=True)
        (self.root / "ros2_ws/build/output").write_text("cache")
        (self.root / "UR5/.editable_build/notes_draft").mkdir(parents=True)
        (self.root / "UR5/.editable_build/notes_draft/paper.pptx").write_bytes(b"research")
        paths = {p for p, _ in self.m.walk()}
        self.assertNotIn(".env", paths)
        self.assertIn(".env.example", paths)
        self.assertNotIn("ros2_ws/build/output", paths)
        self.assertIn("UR5/.editable_build/notes_draft/paper.pptx", paths)

    def test_path_traversal_rejected(self):
        for path in ("../outside", "/tmp/outside", "a\nb", "."):
            with self.assertRaises(ValueError):
                safe_relative(path)

    def test_inventory_respects_ignore_but_snapshot_preserves_payload(self):
        (self.root / ".gitignore").write_text("local-cache.json\n")
        (self.root / "local-cache.json").write_text('{"fixture": true}\n')
        result = self.m.inventory(write=True)
        self.assertIn("local-cache.json", result["git_ignored_candidates"])
        self.assertNotIn("local-cache.json", (self.root / "maintenance/git_paths.txt").read_text().splitlines())
        self.assertIn("local-cache.json", {p for p, _ in self.m.walk()})

    def test_snapshot_restore_and_immutability(self):
        first = self.m.backup()
        self.assertEqual(first["status"], "PASS")
        first_manifest = (self.m.snapshot_dir() / "manifest.json").read_bytes()
        self.assertEqual(self.m.verify()["git_bundles_checked"], 1)
        second = self.m.backup()
        self.assertGreater(second["unchanged_file_reuses"], 0)
        self.assertEqual(second["new_object_bytes"], 0)
        (self.root / "source.py").write_text("value = 2\n")
        self.assertEqual((self.m.snapshot_dir(first["snapshot"]) / "manifest.json").read_bytes(), first_manifest)
        target = Path(self.temp.name) / "restored"
        result = self.m.restore(target, ["source.py", "result.mat"], first["snapshot"])
        self.assertEqual(result["restored_files"], 2)
        self.assertEqual((target / "source.py").read_text(), "value = 1\n")
        self.assertEqual(sha256(target / "result.mat"), sha256(self.root / "result.mat"))
        (target / "source.py").write_text("edited restored copy\n")
        self.assertEqual(self.m.verify(first["snapshot"])["status"], "PASS")
        with self.assertRaises(ValueError):
            self.m.restore(target, ["source.py"])
        with self.assertRaises(ValueError):
            self.m.restore(self.root / "new", ["source.py"])

    def test_corruption_detected(self):
        self.m.backup()
        manifest = json.loads((self.m.snapshot_dir() / "manifest.json").read_text())
        item = next(v for v in manifest["files"] if v["path"] == "result.mat")
        obj = self.m.store / "objects" / item["sha256"][:2] / item["sha256"]
        obj.chmod(0o600)
        obj.write_bytes(b"damaged")
        with self.assertRaises(RuntimeError):
            self.m.verify()
        with self.assertRaises(RuntimeError):
            self.m.restore(Path(self.temp.name) / "not-created", ["result.mat"])
        self.assertFalse((Path(self.temp.name) / "not-created").exists())

    def test_symlink_never_followed(self):
        external = Path(self.temp.name) / "outside"
        external.mkdir()
        (external / "private.py").write_text("not in scope\n")
        (self.root / "linked").symlink_to(external, target_is_directory=True)
        self.assertIn("linked", {p for p, _ in self.m.walk()})
        self.assertNotIn("linked/private.py", {p for p, _ in self.m.walk()})
        self.m.backup()
        with self.assertRaises(ValueError):
            self.m.restore(Path(self.temp.name) / "restored", ["linked"])

    def test_existing_stage_not_changed(self):
        (self.root / "source.py").write_text("value = 3\n")
        self.git("add", "source.py")
        before = self.git("diff", "--cached").stdout
        with self.assertRaises(RuntimeError):
            self.m.checkpoint(["HANDOFF.md"], "Do not commit fixture", "Fixture test")
        self.assertEqual(before, self.git("diff", "--cached").stdout)

    def test_project_change_during_backup_fails_closed(self):
        original_walk = self.m.walk
        count = 0

        def changing_walk():
            nonlocal count
            count += 1
            if count == 3:  # scan, copy, final consistency check
                (self.root / "source.py").write_text("changed during copy\n")
            yield from original_walk()

        self.m.walk = changing_walk
        with self.assertRaises(RuntimeError):
            self.m.backup()
        self.assertIsNone(self.m.latest())


if __name__ == "__main__":
    unittest.main()
