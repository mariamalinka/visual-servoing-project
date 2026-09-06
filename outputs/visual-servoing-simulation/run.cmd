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
if /i "%~1"=="--verify" goto verify
if /i "%~1"=="--trial" goto trial
if /i "%~1"=="--benchmark" goto benchmark
if /i "%~1"=="--report" goto report
"%VSERVO_PYTHON%" "%~dp0app.py" %*
if errorlevel 1 goto failure
exit /b 0
:verify
"%VSERVO_PYTHON%" -m unittest discover -s "%~dp0tests" -v
if errorlevel 1 goto failure
"%VSERVO_PYTHON%" "%~dp0verify.py"
if errorlevel 1 goto failure
exit /b 0
:trial
"%VSERVO_PYTHON%" "%~dp0run_alignment.py"
if errorlevel 1 goto failure
exit /b 0
:benchmark
"%VSERVO_PYTHON%" "%~dp0benchmark.py"
if errorlevel 1 goto failure
exit /b 0
:report
"%VSERVO_PYTHON%" "%~dp0analyze_benchmark.py"
if errorlevel 1 goto failure
exit /b 0
:failure
echo.
echo VServo could not finish. Please keep the error above for troubleshooting.
pause
exit /b 1
