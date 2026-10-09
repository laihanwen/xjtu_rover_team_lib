param([int]$WebPort=8768)
$ErrorActionPreference='Stop'
$taskRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskCandidates=@(
    (Join-Path $env:LOCALAPPDATA 'xjtu-auv/console-venv/Scripts/python.exe'),
    (Join-Path $env:USERPROFILE 'AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/xjtu-auv/console-venv/Scripts/python.exe')
)
$taskPython=$taskCandidates | Where-Object {Test-Path -LiteralPath $_} | Select-Object -First 1
if(!$taskPython){throw 'Project console Python environment not found'}
$taskUrl="http://127.0.0.1:$WebPort"
try { $taskExisting=Invoke-RestMethod "$taskUrl/collection" -TimeoutSec 2 } catch { $taskExisting=$null }
if(!$taskExisting){
    $taskLogs=Join-Path $taskRoot ('build/data-collection/'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
    New-Item -ItemType Directory -Force $taskLogs | Out-Null
    $taskProcess=Start-Process -FilePath $taskPython -ArgumentList @(
        ('"'+(Join-Path $PSScriptRoot 'trial_control_web.py')+'"'),'--observe-only','--web-port',"$WebPort"
    ) -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $taskLogs 'stdout.log') -RedirectStandardError (Join-Path $taskLogs 'stderr.log')
    for($taskAttempt=0;$taskAttempt -lt 30;$taskAttempt++){
        if($taskProcess.HasExited){throw "Console exited: $taskLogs"}
        try { $taskExisting=Invoke-RestMethod "$taskUrl/collection" -TimeoutSec 1;break } catch {Start-Sleep -Milliseconds 200}
    }
    if(!$taskExisting){throw "Console unavailable: $taskLogs"}
}
Start-Process $taskUrl
Write-Output "Read-only collection console: $taskUrl"
