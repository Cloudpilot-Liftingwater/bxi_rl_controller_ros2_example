#!/usr/bin/env python3
"""Regression tests for commit-only public release exports."""

import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools.sanitize_release import (
    PUBLIC_DEV_ONLY_PATHS,
    copy_release_tree,
    repository_relative_argument,
    resolve_source_commit,
)


def run_git(repository: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repository), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip()


class CommitOnlyExportTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.repository = Path(self.temporary_directory.name) / "source"
        self.repository.mkdir()
        run_git(self.repository, "init", "--quiet")
        run_git(self.repository, "config", "user.name", "Release Test")
        run_git(self.repository, "config", "user.email", "release-test@example.invalid")

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def commit_all(self, message: str) -> str:
        run_git(self.repository, "add", "--all")
        run_git(self.repository, "commit", "--quiet", "-m", message)
        return run_git(self.repository, "rev-parse", "HEAD")

    def test_export_is_exactly_the_requested_commit(self) -> None:
        tracked = self.repository / "tracked.txt"
        unicode_executable = self.repository / "工具" / "启动.sh"
        tracked.write_text("commit one\n", encoding="utf-8")
        unicode_executable.parent.mkdir()
        unicode_executable.write_text("#!/bin/sh\necho commit-one\n", encoding="utf-8")
        unicode_executable.chmod(0o755)
        first_commit = self.commit_all("first")

        tracked.write_text("commit two\n", encoding="utf-8")
        (self.repository / "second-commit.txt").write_text(
            "only in the second commit\n", encoding="utf-8"
        )
        self.commit_all("second")

        # Dirty tracked and untracked content must never leak into the export.
        tracked.write_text("dirty worktree\n", encoding="utf-8")
        (self.repository / "untracked.txt").write_text(
            "untracked worktree\n", encoding="utf-8"
        )

        # Exercise the important case where --out is below the source repository.
        output = self.repository / "dist" / "public_release"
        output.mkdir(parents=True)
        (output / "stale.txt").write_text("stale output\n", encoding="utf-8")

        resolved_commit = resolve_source_commit(self.repository, first_commit)
        copy_release_tree(self.repository, output, resolved_commit)

        exported_files = {
            path.relative_to(output).as_posix()
            for path in output.rglob("*")
            if path.is_file()
        }
        self.assertEqual(exported_files, {"tracked.txt", "工具/启动.sh"})
        self.assertEqual((output / "tracked.txt").read_text(encoding="utf-8"), "commit one\n")
        self.assertEqual(
            (output / "工具" / "启动.sh").read_text(encoding="utf-8"),
            "#!/bin/sh\necho commit-one\n",
        )
        exported_mode = stat.S_IMODE((output / "工具" / "启动.sh").stat().st_mode)
        self.assertEqual(exported_mode, 0o755)
        self.assertFalse((output / "second-commit.txt").exists())
        self.assertFalse((output / "untracked.txt").exists())
        self.assertFalse((output / "stale.txt").exists())

    def test_symlink_in_commit_is_rejected(self) -> None:
        target = self.repository / "target.txt"
        target.write_text("target\n", encoding="utf-8")
        os.symlink("target.txt", self.repository / "link.txt")
        source_commit = self.commit_all("symlink")

        with self.assertRaisesRegex(RuntimeError, "unsupported entry.*link.txt"):
            copy_release_tree(
                self.repository,
                self.repository / "dist" / "public_release",
                source_commit,
            )

    def test_release_input_paths_cannot_escape_repository(self) -> None:
        inside = self.repository / "config" / "release.yaml"
        inside.parent.mkdir()
        inside.write_text("protected_states: {}\n", encoding="utf-8")
        self.assertEqual(
            repository_relative_argument(self.repository, inside, "manifest"),
            Path("config/release.yaml"),
        )

        outside = Path(self.temporary_directory.name) / "outside.yaml"
        outside.write_text("protected_states: {}\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "must be inside the source repository"):
            repository_relative_argument(self.repository, outside, "manifest")

    def test_output_cannot_be_repository_or_its_parent(self) -> None:
        (self.repository / "tracked.txt").write_text("tracked\n", encoding="utf-8")
        source_commit = self.commit_all("source")
        for unsafe_output in (self.repository, self.repository.parent):
            with self.subTest(output=unsafe_output):
                with self.assertRaisesRegex(ValueError, "output directory must not"):
                    copy_release_tree(self.repository, unsafe_output, source_commit)

    def test_commercial_runtime_excludes_development_only_assets(self) -> None:
        excluded = {path.as_posix() for path in PUBLIC_DEV_ONLY_PATHS}
        self.assertIn("docs", excluded)
        self.assertIn("src/bxi_example_py_elf3/test", excluded)
        self.assertIn(
            "src/bxi_example_py_elf3/data/mujoco_simulation",
            excluded,
        )
        self.assertIn(
            "src/bxi_example_py_elf3/data/sonic_robot_model/"
            "elf3_dof29_hand/urdf/meshes",
            excluded,
        )
        self.assertNotIn(
            "src/bxi_example_py_elf3/data/sonic_robot_model/"
            "elf3_dof29_hand/urdf/elf3.urdf",
            excluded,
        )
        self.assertNotIn(
            "src/bxi_example_py_elf3/data/sonic_model/"
            "elf3_step28800_smpl/model_step_028800_smpl.onnx",
            excluded,
        )
        self.assertNotIn(
            "src/bxi_example_py_elf3/data/sonic_reference/"
            "elf3_pico_stand_clean_001/stream_reference.npz",
            excluded,
        )


if __name__ == "__main__":
    unittest.main()
