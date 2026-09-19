@echo off
setlocal
call "%~dp0outputs\visual-servoing-simulation\setup.cmd" %*
exit /b %errorlevel%
