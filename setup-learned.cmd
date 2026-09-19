@echo off
setlocal
call "%~dp0outputs\visual-servoing-simulation\setup-learned.cmd" %*
exit /b %errorlevel%
