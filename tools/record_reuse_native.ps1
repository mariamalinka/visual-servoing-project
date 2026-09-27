param(
    [ValidateSet('learned', 'natural')][string]$Mode = 'learned',
    [ValidatePattern('^20260924-native-(learned|natural)-[0-9]{3}$')][string]$OutputName = '20260924-native-learned-001'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$appRoot = Join-Path $projectRoot 'outputs/visual-servoing-simulation'
$latencyRoot = Join-Path $appRoot 'results/latency'
$outputPath = Join-Path $latencyRoot $OutputName
$pythonPath = Join-Path $appRoot '.venv/Scripts/python.exe'
$nativeScript = Join-Path $PSScriptRoot 'record_reuse_native.py'
Set-Location -LiteralPath $projectRoot
try {
    & $pythonPath -B $nativeScript --mode $Mode --output $outputPath *> (Join-Path $latencyRoot ($OutputName + '.console.txt'))
    $runExit = $LASTEXITCODE
} catch {
    $_ | Out-String | Set-Content -LiteralPath (Join-Path $latencyRoot ($OutputName + '.launcher-error.txt'))
    $runExit = 1
}
$runExit | Set-Content -LiteralPath (Join-Path $latencyRoot ($OutputName + '.exit.txt'))
exit $runExit
