param(
    [ValidateSet('baseline', 'synq', 'both')]
    [string]$Mode = 'both'
)
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$synqRoot = if ($env:SYNQ_ROOT) { $env:SYNQ_ROOT } else { (Resolve-Path (Join-Path $projectRoot '..')).Path }
$webotsHome = if ($env:WEBOTS_HOME) { $env:WEBOTS_HOME } else { Join-Path $env:LOCALAPPDATA 'Programs\Webots' }
$webotsExe = Join-Path $webotsHome 'msys64\mingw64\bin\webots.exe'
$pythonExe = Join-Path $synqRoot '.venv\Scripts\python.exe'
$world = Join-Path $projectRoot 'worlds\synq_three_robot.wbt'
foreach ($path in @($webotsExe, $pythonExe, $world)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Required path missing: $path" }
}
$env:SYNQ_ROOT = $synqRoot
$env:SYNQ_BATCH = '1'
$env:PATH = (Split-Path -Parent $pythonExe) + ';' + $env:PATH
$modes = if ($Mode -eq 'both') { @('baseline', 'synq') } else { @($Mode) }
foreach ($runMode in $modes) {
    $env:MODE = $runMode
    $env:SYNQ_RESULT_FILE = Join-Path $PSScriptRoot ($runMode + '_result.json')
    $previousWrite = if (Test-Path -LiteralPath $env:SYNQ_RESULT_FILE) { (Get-Item -LiteralPath $env:SYNQ_RESULT_FILE).LastWriteTimeUtc } else { $null }
    Write-Host "Running $runMode on the same Webots world..."
    & $webotsExe --batch --mode=fast --port=1238 --stdout --stderr $world
    if ($LASTEXITCODE -ne 0) { throw "Webots $runMode run failed with exit code $LASTEXITCODE" }
    if (-not (Test-Path -LiteralPath $env:SYNQ_RESULT_FILE)) { throw "Missing result: $env:SYNQ_RESULT_FILE" }
    $resultFile = Get-Item -LiteralPath $env:SYNQ_RESULT_FILE
    if ($null -ne $previousWrite -and $resultFile.LastWriteTimeUtc -le $previousWrite) { throw "Result file was not refreshed: $env:SYNQ_RESULT_FILE" }
    $result = Get-Content -LiteralPath $env:SYNQ_RESULT_FILE -Raw | ConvertFrom-Json
    if ($result.mode -ne $runMode -or $null -eq $result.makespan -or -not $result.physically_complete -or -not $result.peer_exchange_complete) { throw "Incomplete $runMode experiment result" }
}
if ($Mode -eq 'both') {
    & $pythonExe -B (Join-Path $PSScriptRoot 'compare_results.py') (Join-Path $PSScriptRoot 'baseline_result.json') (Join-Path $PSScriptRoot 'synq_result.json')
    if ($LASTEXITCODE -ne 0) { throw 'Comparison failed' }
}
