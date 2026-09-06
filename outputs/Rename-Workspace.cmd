@echo off
setlocal
rem Run after closing Codex. Work outside the directory being renamed.
set "VSERVO_RENAME_PARENT=%~dp0..\.."
set "VSERVO_RENAME_CHECK=0"
if /i "%~1"=="--check" set "VSERVO_RENAME_CHECK=1"
cd /d "%VSERVO_RENAME_PARENT%"
powershell.exe -NoProfile -Command "$ErrorActionPreference = 'Stop'; $parent = [IO.Path]::GetFullPath($env:VSERVO_RENAME_PARENT).TrimEnd([IO.Path]::DirectorySeparatorChar); $source = [IO.Path]::GetFullPath((Join-Path $parent 'i-want-to-do-that-project')); $target = [IO.Path]::GetFullPath((Join-Path $parent 'visual-servoing-project')); if (-not $source.StartsWith($parent + [IO.Path]::DirectorySeparatorChar) -or -not $target.StartsWith($parent + [IO.Path]::DirectorySeparatorChar)) { throw 'Unexpected rename path' }; if ((Test-Path -LiteralPath $target) -and -not (Test-Path -LiteralPath $source)) { Write-Host ('Already renamed: ' + $target); exit 0 }; if (Test-Path -LiteralPath $target) { throw 'Destination exists. No files were changed.' }; if (-not (Test-Path -LiteralPath (Join-Path $source 'outputs/visual-servoing-simulation/scene.xml'))) { throw 'Expected project was not found. No files were changed.' }; Write-Host ('From: ' + $source); Write-Host ('To:   ' + $target); if ($env:VSERVO_RENAME_CHECK -eq '1') { Write-Host 'Path checks passed. Check-only mode: nothing renamed.'; exit 0 }; try { Rename-Item -LiteralPath $source -NewName 'visual-servoing-project'; Write-Host 'Rename complete. Open outputs/visual-servoing-simulation/run.cmd inside the renamed folder.' } catch { Write-Host 'Rename could not finish. Close Codex and any terminal using the workspace, then try again.'; Write-Host $_.Exception.Message; exit 1 }"
set "VSERVO_RENAME_EXIT=%ERRORLEVEL%"
if "%VSERVO_RENAME_CHECK%"=="1" exit /b %VSERVO_RENAME_EXIT%
pause
exit /b %VSERVO_RENAME_EXIT%
