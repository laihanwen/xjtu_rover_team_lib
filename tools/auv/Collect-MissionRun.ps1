param(
  [string]$HostName = 'pi@192.168.137.150',
  [string]$RunId = '',
  [string]$IdentityFile = '',
  [string]$ManualGrid = '',
  [string]$OutputRoot = 'logs/mission-runs',
  [string]$LocalRun = ''
)
$ErrorActionPreference = 'Stop'
$scriptPath = Join-Path $PSScriptRoot 'collect_mission_run.py'
$python = Get-Command python -ErrorAction Stop
$arguments = @($scriptPath, '--output-root', $OutputRoot)
if ($LocalRun) { $arguments += @('--local-run', $LocalRun) }
else { $arguments += @('--host', $HostName) }
if ($RunId) { $arguments += @('--run-id', $RunId) }
if ($IdentityFile) { $arguments += @('--identity', $IdentityFile) }
if ($ManualGrid) { $arguments += @('--manual-grid', $ManualGrid) }
& $python.Source @arguments
exit $LASTEXITCODE
