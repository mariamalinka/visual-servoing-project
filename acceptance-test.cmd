@echo off
rem SIFT + Learned GPU acceptance test (about 30 minutes). See docs\ACCEPTANCE_TEST.md.
rem Exit code: 0 PASS, 1 FAIL, 2 INVALID or INCOMPLETE. Extra arguments are passed through
rem (for example --smoke for a 4-minute harness check, or --long for the under-1-hour run).
setlocal
set "PY=%~dp0outputs\visual-servoing-simulation\.venv\Scripts\python.exe"
if not exist "%PY%" (
  echo Python environment not found: %PY%
  echo Run setup.cmd and setup-learned.cmd --cuda first.
  exit /b 2
)
"%PY%" -B "%~dp0tools\run_acceptance_test.py" %*
exit /b %errorlevel%
