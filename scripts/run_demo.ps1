param(
    [Parameter(Mandatory=$true)][ValidateSet('m1','eight','p3','m2','m3','m4','m5','m6','c1','phs')][string]$Demo,
    [string]$Case = '',
    [double]$Speed = 1,
    [switch]$Loop,
    [switch]$Technical
)
$showcaseRoot = Split-Path -Parent $PSScriptRoot
$showcasePython = Join-Path $showcaseRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $showcasePython)) { throw 'Create .venv and install requirements.txt first (see README).' }
$viewerArgs = @((Join-Path $showcaseRoot 'viewer\showcase.py'), $Demo, '--speed', [string]$Speed)
if ($Case) { $viewerArgs += @('--case', $Case) }
if ($Loop) { $viewerArgs += '--loop' }
if ($Technical) { $viewerArgs += '--technical' }
& $showcasePython @viewerArgs
exit $LASTEXITCODE
