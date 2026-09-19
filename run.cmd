@echo off
setlocal
call "%~dp0outputs\visual-servoing-simulation\run.cmd" %*
exit /b %errorlevel%
