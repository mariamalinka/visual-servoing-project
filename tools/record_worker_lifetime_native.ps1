$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot
$appRoot = Join-Path $projectRoot 'outputs/visual-servoing-simulation'
$outputRoot = Join-Path $appRoot 'results/latency/20260925-lifetime-native'
& (Join-Path $appRoot '.venv/Scripts/python.exe') -B (Join-Path $PSScriptRoot 'diagnose_worker_lifetime.py') --native --output $outputRoot *> ($outputRoot + '.console.txt')
$runExit = $LASTEXITCODE
$runExit | Set-Content -LiteralPath ($outputRoot + '.exit.txt')
exit $runExit
