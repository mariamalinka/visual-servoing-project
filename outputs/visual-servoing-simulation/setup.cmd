@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto install
py -3.12 --version >nul 2>nul
if not errorlevel 1 goto pylauncher
python --version >nul 2>nul
if not errorlevel 1 goto pythonpath
set "VSERVO_BOOTSTRAP=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%VSERVO_BOOTSTRAP%" goto bundled
echo Python was not found. Install Python 3.12 from python.org, then run setup.cmd again.
pause
exit /b 1
:pylauncher
py -3.12 -m venv .venv
if errorlevel 1 goto failure
goto install
:pythonpath
python -m venv .venv
if errorlevel 1 goto failure
goto install
:bundled
"%VSERVO_BOOTSTRAP%" -m venv .venv
if errorlevel 1 goto failure
:install
".venv\Scripts\python.exe" -m pip install --disable-pip-version-check -r requirements.txt
if errorlevel 1 goto failure
echo.
echo Setup complete. Double-click run.cmd to open the simulation.
exit /b 0
:failure
echo.
echo Setup failed. Please keep the error above for troubleshooting.
pause
exit /b 1
