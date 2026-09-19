@echo off
setlocal
cd /d "%~dp0"
set "VSERVO_PYTHON=%~dp0.venv\Scripts\python.exe"
if exist "%VSERVO_PYTHON%" goto ready
set "VSERVO_PYTHON=%~dp0..\..\work\vservo-venv\Scripts\python.exe"
if exist "%VSERVO_PYTHON%" goto ready
call "%~dp0setup.cmd"
if errorlevel 1 exit /b 1
set "VSERVO_PYTHON=%~dp0.venv\Scripts\python.exe"
:ready
"%VSERVO_PYTHON%" "%~dp0setup_learned.py" %*
if errorlevel 1 goto failure
echo Learned matching is ready. Run run.cmd --learned.
exit /b 0
:failure
echo Learned setup failed. Keep the error above for troubleshooting.
pause
exit /b 1
