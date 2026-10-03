@echo off
setlocal
set "VSERVO_CALLER_DIR=%CD%"
cd /d "%~dp0"
set "VSERVO_PYTHON=%~dp0.venv\Scripts\python.exe"
if exist "%VSERVO_PYTHON%" goto ready
set "VSERVO_PYTHON=%~dp0..\..\work\vservo-venv\Scripts\python.exe"
if exist "%VSERVO_PYTHON%" goto ready
call "%~dp0setup.cmd"
if errorlevel 1 exit /b 1
set "VSERVO_PYTHON=%~dp0.venv\Scripts\python.exe"
:ready
if /i "%~1"=="--latency-study" goto latency_study
if /i "%~1"=="--stop-response" goto stop_response
if /i "%~1"=="--fault-campaign" goto fault_campaign
if /i "%~1"=="--accuracy-study" goto accuracy_study
if /i "%~1"=="--collision-study" goto collision_study
if /i "%~1"=="--robustness" goto robustness
if /i "%~1"=="--delay-study" goto delay_study
if /i "%~1"=="--verify" goto verify
if /i "%~1"=="--trial" goto trial
if /i "%~1"=="--benchmark" goto benchmark
if /i "%~1"=="--report" goto report
if /i "%~1"=="--recovery" goto recovery
if /i "%~1"=="--compare" goto compare
if /i "%~1"=="--compare-report" goto compare_report
if /i "%~1"=="--joint-study" goto joint_study
if /i "%~1"=="--startup-study" goto startup_study
if /i "%~1"=="--coverage-study" goto coverage_study
if /i "%~1"=="--natural-study" goto natural_study
if /i "%~1"=="--learned-study" goto learned_study
if /i "%~1"=="--gain-study" goto gain_study
if /i "%~1"=="--gain-report" goto gain_report
"%VSERVO_PYTHON%" "%~dp0app.py" %*
if errorlevel 1 goto failure
exit /b 0
:latency_study
pushd "%VSERVO_CALLER_DIR%"
"%VSERVO_PYTHON%" "%~dp0run_latency_study.py" %*
set "VSERVO_EXPERIMENT_EXIT=%errorlevel%"
popd
exit /b %VSERVO_EXPERIMENT_EXIT%
:fault_campaign
pushd "%VSERVO_CALLER_DIR%"
"%VSERVO_PYTHON%" "%~dp0run_fault_campaign.py" %*
set "VSERVO_EXPERIMENT_EXIT=%errorlevel%"
popd
exit /b %VSERVO_EXPERIMENT_EXIT%
:stop_response
pushd "%VSERVO_CALLER_DIR%"
"%VSERVO_PYTHON%" "%~dp0run_stop_response.py" %*
set "VSERVO_EXPERIMENT_EXIT=%errorlevel%"
popd
exit /b %VSERVO_EXPERIMENT_EXIT%
:accuracy_study
pushd "%VSERVO_CALLER_DIR%"
"%VSERVO_PYTHON%" "%~dp0run_accuracy_study.py" %*
set "VSERVO_EXPERIMENT_EXIT=%errorlevel%"
popd
exit /b %VSERVO_EXPERIMENT_EXIT%
:collision_study
pushd "%VSERVO_CALLER_DIR%"
"%VSERVO_PYTHON%" "%~dp0run_collision_study.py" %*
set "VSERVO_EXPERIMENT_EXIT=%errorlevel%"
popd
exit /b %VSERVO_EXPERIMENT_EXIT%
:robustness
pushd "%VSERVO_CALLER_DIR%"
"%VSERVO_PYTHON%" "%~dp0run_camera_robustness.py" %*
set "VSERVO_EXPERIMENT_EXIT=%errorlevel%"
popd
exit /b %VSERVO_EXPERIMENT_EXIT%
:delay_study
"%VSERVO_PYTHON%" "%~dp0run_camera_delay_study.py" %*
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
:recovery
"%VSERVO_PYTHON%" "%~dp0run_recovery.py"
if errorlevel 1 goto failure
exit /b 0
:compare
"%VSERVO_PYTHON%" "%~dp0compare_controllers.py"
if errorlevel 1 goto failure
exit /b 0
:compare_report
"%VSERVO_PYTHON%" "%~dp0analyze_comparison.py"
if errorlevel 1 goto failure
exit /b 0
:joint_study
"%VSERVO_PYTHON%" "%~dp0run_joint_limit_study.py"
if errorlevel 1 goto failure
exit /b 0
:learned_study
"%VSERVO_PYTHON%" "%~dp0run_learned_study.py"
if errorlevel 1 goto failure
exit /b 0
:natural_study
"%VSERVO_PYTHON%" "%~dp0run_natural_image_study.py"
if errorlevel 1 goto failure
exit /b 0
:coverage_study
"%VSERVO_PYTHON%" "%~dp0run_search_coverage.py"
if errorlevel 1 goto failure
exit /b 0
:startup_study
"%VSERVO_PYTHON%" "%~dp0run_startup_search.py"
if errorlevel 1 goto failure
exit /b 0
:gain_study
"%VSERVO_PYTHON%" "%~dp0run_gain_study.py"
if errorlevel 1 goto failure
exit /b 0
:gain_report
"%VSERVO_PYTHON%" "%~dp0analyze_gain_study.py"
if errorlevel 1 goto failure
exit /b 0
:failure
echo.
echo VServo could not finish. Please keep the error above for troubleshooting.
pause
exit /b 1
