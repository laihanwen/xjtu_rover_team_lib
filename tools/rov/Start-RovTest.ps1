param(
    [switch]$Flash,
    [string]$PiHost = '192.168.137.150',
    [int]$WebPort = 8767,
    [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskUrl = "http://127.0.0.1:$WebPort"
$taskOldPythonPath = $env:PYTHONPATH
Push-Location $taskRoot
try {
    if ($Flash) {
        # Refuse maintenance of a known running vehicle. Never send ARM.
        $taskState = $null
        try { $taskState = Invoke-RestMethod "$taskUrl/state" -TimeoutSec 2 } catch {}
        if ($taskState -and $taskState.telemetry.armed) { throw 'Press STOP / DISARM before flashing.' }
        $taskEntry = Join-Path $PSScriptRoot 'trial_control_web.py'
        Get-CimInstance Win32_Process | Where-Object {
            $_.Name -match '^python(w)?\.exe$' -and
            ($_.CommandLine -like "*$taskEntry*" -or $_.CommandLine -like '*tools/rov/trial_control_web.py*')
        } | ForEach-Object { Stop-Process -Id $_.ProcessId }
        & (Join-Path $PSScriptRoot 'Maintain-Rov.ps1') build
        & (Join-Path $PSScriptRoot 'Maintain-Rov.ps1') flash
    }
    $taskState = $null
    try { $taskState = Invoke-RestMethod "$taskUrl/state" -TimeoutSec 2 } catch {}
    if (!$taskState) {
        $env:PYTHONPATH = Join-Path $taskRoot 'build/joystick-deps'
        $env:SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS = '1'
        $taskLogs = Join-Path $taskRoot ('build/rov-test/' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
        New-Item -ItemType Directory -Force $taskLogs | Out-Null
        $taskProcess = Start-Process -FilePath (Get-Command python).Source -ArgumentList @(
            ('"' + (Join-Path $PSScriptRoot 'trial_control_web.py') + '"'),
            '--host',$PiHost,'--web-port',"$WebPort",'--diagnostic-log'
        ) -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
          -RedirectStandardOutput (Join-Path $taskLogs 'console.stdout.log') `
          -RedirectStandardError (Join-Path $taskLogs 'console.stderr.log')
        for ($taskAttempt=0; $taskAttempt -lt 30; $taskAttempt++) {
            if ($taskProcess.HasExited) { throw "Console exited; inspect $taskLogs" }
            try { $taskState=Invoke-RestMethod "$taskUrl/state" -TimeoutSec 1; break } catch {}
            Start-Sleep -Milliseconds 200
        }
        if (!$taskState) { throw "Console did not start; inspect $taskLogs" }
        Write-Host "Console PID: $($taskProcess.Id); startup logs: $taskLogs"
    } else { Write-Host 'Reusing the existing console; no second controller started.' }
    if (!$NoBrowser) { Start-Process $taskUrl }
    Write-Host "Console: $taskUrl"
    Write-Host "Controller: $($taskState.controller_connected); Pi link: $($taskState.connected)"
    Write-Host "Level calibrated: $($taskState.telemetry.level_calibrated); ARM: $($taskState.telemetry.armed)"
    Write-Host 'After MCU restart: place level on shore, check the confirmation, then calibrate.'
    Write-Host 'For water testing: centered sticks + left shoulder permission, then click ARM.'
    Write-Host 'STOP / Esc stops control. No ARM or calibration commands are sent by this script.'
} finally {
    $env:PYTHONPATH = $taskOldPythonPath
    Pop-Location
}
