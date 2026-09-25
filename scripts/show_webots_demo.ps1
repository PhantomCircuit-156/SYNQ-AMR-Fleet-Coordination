param(
    [ValidateSet('baseline', 'synq')]
    [string]$Mode = 'synq',
    [switch]$Paused
)
# Interactive demo only. Benchmark result files are written by the experiment runner.
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$webotsHome = if ($env:WEBOTS_HOME) { $env:WEBOTS_HOME } else { Join-Path $env:LOCALAPPDATA 'Programs\Webots' }
$webotsExe = Join-Path $webotsHome 'msys64\mingw64\bin\webots.exe'
$world = Join-Path $repoRoot 'webots\worlds\synq_three_robot.wbt'
$pythonDir = Join-Path $repoRoot '.venv\Scripts'
foreach ($path in @($webotsExe, $world, (Join-Path $pythonDir 'python.exe'))) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Required path missing: $path" }
}
$names = @('SYNQ_ROOT', 'SYNQ_BATCH', 'SYNQ_RESULT_FILE', 'SYNQ_CAPTURE_DIR', 'MODE', 'PATH')
$previous = @{}
foreach ($name in $names) { $previous[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
try {
    $env:SYNQ_ROOT = $repoRoot
    $env:SYNQ_BATCH = '0'
    $env:MODE = $Mode
    $env:PATH = $pythonDir + ';' + $env:PATH
    Remove-Item Env:SYNQ_RESULT_FILE -ErrorAction SilentlyContinue
    Remove-Item Env:SYNQ_CAPTURE_DIR -ErrorAction SilentlyContinue
    $playback = if ($Paused) { 'pause' } else { 'realtime' }
    & $webotsExe "--mode=$playback" --stdout --stderr $world
    if ($LASTEXITCODE -ne 0) { throw "Webots exited with code $LASTEXITCODE" }
} finally {
    foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process') }
}
