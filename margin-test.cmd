@echo off
rem Latency-margin test for SIFT and Learned GPU (about 15-20 minutes). See docs\MARGIN_TEST.md.
rem Exit code: 0 measured, 2 INVALID or INCOMPLETE. Extra arguments are passed through
rem (for example --smoke for a 2-minute setup check).
setlocal
set "PY=%~dp0outputs\visual-servoing-simulation\.venv\Scripts\python.exe"
if not exist "%PY%" (
  echo Python environment not found: %PY%
  echo Run setup.cmd and setup-learned.cmd --cuda first.
  exit /b 2
)
"%PY%" -B "%~dp0tools\run_margin_test.py" %*
exit /b %errorlevel%
