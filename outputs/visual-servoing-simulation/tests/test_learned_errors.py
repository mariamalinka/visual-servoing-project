"""Regression coverage for unreadable wheels and errors hidden by the GUI."""
import builtins
import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import app
from learned_perception import LearnedUnavailable, load_dependencies, unavailable
import setup_learned


class LearnedErrorTests(unittest.TestCase):
    def test_import_failures_retain_cause_and_actionable_details(self):
        original_import = builtins.__import__
        failures = [PermissionError(13, "Permission denied", "torch/__init__.py"),
                    ModuleNotFoundError("No module named 'torch'"),
                    RuntimeError("operator torchvision::nms does not exist")]
        for failure in failures:
            def failing_import(name, *args, **kwargs):
                if name == "torch":
                    raise failure
                return original_import(name, *args, **kwargs)
            with self.subTest(failure=failure), patch("builtins.__import__", side_effect=failing_import):
                with self.assertRaises(LearnedUnavailable) as caught:
                    load_dependencies()
                self.assertIs(caught.exception.__cause__, failure)
                self.assertIn(type(failure).__name__, str(caught.exception))
                self.assertIn(str(failure), str(caught.exception))
                self.assertIn("setup-learned.cmd", str(caught.exception))

    def report_failure(self):
        try:
            try:
                raise PermissionError(13, "Permission denied", "torch/__init__.py")
            except PermissionError as exc:
                raise unavailable("import", exc) from exc
        except LearnedUnavailable as exc:
            return app.report_perception_error("learned", exc)

    def test_gui_reports_root_cause_and_saves_complete_chain_and_interpreter(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(app, "ROOT", Path(folder)), \
             contextlib.redirect_stderr(io.StringIO()) as stderr:
            message = self.report_failure()
            self.assertIn("Learned import: PermissionError (torch/__init__.py)", message)
            self.assertIn("logs/learned-matcher.log", message)
            details = (Path(folder) / "logs/learned-matcher.log").read_text(encoding="utf-8")
            for text in ("PermissionError", "torch/__init__.py", "LearnedUnavailable",
                         "setup-learned.cmd --fresh", sys.executable, "Traceback"):
                self.assertIn(text, details)
                self.assertIn(text, stderr.getvalue())

    def test_unwritable_log_does_not_hide_original_error_or_crash(self):
        with patch("pathlib.Path.mkdir", side_effect=PermissionError("log denied")), \
             contextlib.redirect_stderr(io.StringIO()) as stderr:
            message = self.report_failure()
        self.assertIn("PermissionError (torch/__init__.py)", message)
        self.assertIn("console (log write failed)", message)
        self.assertIn("log denied", stderr.getvalue())


class LearnedInstallerTests(unittest.TestCase):
    def test_unreadable_runtime_is_not_mistaken_for_installed_success(self):
        failure = subprocess.CompletedProcess([], 1, "", "PermissionError: torch/__init__.py")
        with patch.object(sys, "argv", ["setup_learned.py"]), \
             patch.object(setup_learned, "probe_runtime", return_value=failure), \
             patch.object(setup_learned.subprocess, "run") as run, \
             contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(SystemExit, "setup-learned.cmd --fresh"):
                setup_learned.main()
            run.assert_not_called()

    def test_broken_import_is_reinstalled_even_when_pip_reports_satisfied(self):
        failure = subprocess.CompletedProcess([], 1, "", "RuntimeError: broken torch")
        with patch.object(sys, "argv", ["setup_learned.py"]), \
             patch.object(setup_learned, "probe_runtime", return_value=failure), \
             patch.object(setup_learned.subprocess, "run") as run, \
             contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            setup_learned.main()
        commands = [c.args[0] for c in run.call_args_list]
        reinstall = next(command for command in commands if "--force-reinstall" in command)
        self.assertIn("--no-deps", reinstall)
        self.assertTrue(any(setup_learned.PROBE in command for command in commands))
        self.assertTrue(all(c.kwargs.get("check") for c in run.call_args_list))

    def test_fresh_refuses_to_overwrite_an_existing_environment(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / ".venv"
            destination.mkdir()
            marker = destination / "preserve.txt"
            marker.write_text("existing installation")
            with patch.object(sys, "argv", ["setup_learned.py", "--fresh"]), \
                 patch.object(setup_learned, "ROOT", Path(folder)), \
                 self.assertRaisesRegex(SystemExit, "already exists; preserved"):
                setup_learned.main()
            self.assertEqual(marker.read_text(), "existing installation")


if __name__ == "__main__":
    unittest.main()
