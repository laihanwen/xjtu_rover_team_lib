param(
    [string]$Python,
    [string]$PiHost = '192.168.137.150',
    [string]$PiUser = 'pi',
    [int]$WebPort = 8767,
    [string]$Keil = 'C:/Keil_v5/UV4/UV4.exe',
    [string]$Probe = 'ATK 20190528',
    [string]$Pack = 'C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8',
    [switch]$CheckOnly,
    [switch]$SafetyConfirmed,
    [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskOldPath = $env:PATH
$taskOldPythonPath = $env:PYTHONPATH
$taskOldPassword = $env:AUV_DEPLOY_PASSWORD
Push-Location $taskRoot
try {
    if (!$Python) {
        $taskCandidates = @(
            (Join-Path $env:LOCALAPPDATA 'xjtu-auv/console-venv/Scripts/python.exe'),
            (Join-Path $env:USERPROFILE 'AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/xjtu-auv/console-venv/Scripts/python.exe')
        )
        $Python = $taskCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
    if (!$Python -or !(Test-Path -LiteralPath $Python)) { throw '未找到工具环境；用 -Python 指定项目 Python 可执行文件。' }
    if (!(Test-Path -LiteralPath $Keil)) { throw "未找到 Keil：$Keil" }
    if (!(Test-Path -LiteralPath (Join-Path $Pack 'Flash/STM32F4xx_1024.FLM'))) { throw '缺少 STM32F4 烧录算法包。' }
    $env:PATH = (Split-Path $Python) + [IO.Path]::PathSeparator + $taskOldPath
    $env:PYTHONPATH = ((@('build/debug-deps','build/deploy-ssh','build/joystick-deps') | ForEach-Object { Join-Path $taskRoot $_ }) + @($taskOldPythonPath) | Where-Object { $_ }) -join [IO.Path]::PathSeparator
    & $Python -c 'import paramiko,pyocd,intelhex,pygame'
    if ($LASTEXITCODE -ne 0) { throw '工具依赖检查失败；未执行烧录。' }
    if (!$env:AUV_DEPLOY_PASSWORD -and $PiHost -eq '192.168.137.150' -and $PiUser -eq 'pi') {
        $taskCredentialCandidates = @(
            (Join-Path $env:LOCALAPPDATA 'xjtu-auv/pi-password.dpapi'),
            (Join-Path $env:USERPROFILE 'AppData/Local/xjtu-auv/pi-password.dpapi'),
            (Join-Path $env:USERPROFILE 'AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/xjtu-auv/pi-password.dpapi')
        )
        $taskCredentials = $taskCredentialCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
        if ($taskCredentials) {
            $taskSecret = (Get-Content -LiteralPath $taskCredentials -Raw).Trim() | ConvertTo-SecureString
            $env:AUV_DEPLOY_PASSWORD = ([System.Net.NetworkCredential]::new('', $taskSecret)).Password
        }
    }
    if ($CheckOnly) {
      Write-Host '只读检查树莓派连接与现有 ROV / 摄像头服务……'
      & $Python tools/rov/maintenance.py check --host $PiHost --user $PiUser
      if ($LASTEXITCODE -ne 0) {
        $taskFailure = Get-ChildItem (Join-Path $taskRoot 'build/maintenance') -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
        $taskReasonFile = Join-Path $taskFailure.FullName 'failure.txt'
        $taskReason = if (Test-Path -LiteralPath $taskReasonFile) { Get-Content -LiteralPath $taskReasonFile -Raw } else { '请查看检查输出。' }
        throw "树莓派只读检查失败，未烧录。具体原因：$taskReason 日志：$($taskFailure.FullName)"
      }
    }
    $taskUrl = "http://127.0.0.1:$WebPort"
    $taskState = $null
    try { $taskState = Invoke-RestMethod "$taskUrl/state" -TimeoutSec 2 } catch {}
    $taskOccupied = Get-NetTCPConnection -State Listen -LocalPort $WebPort -ErrorAction SilentlyContinue
    if (!$taskState -and $taskOccupied) { throw '网页端口被其他服务占用；未执行烧录。' }
    if ($taskState) {
        if ($taskState.telemetry.armed) { throw '当前已 ARM，请先 STOP / DISARM。' }
        $taskRecording = Invoke-RestMethod "$taskUrl/recording" -TimeoutSec 3
        if ($taskRecording.active) { throw '请先结束录像，等待保存完成。' }
        if (@($taskRecording.cameras.PSObject.Properties | Where-Object { $_.Value.finished -eq $false }).Count) {
            throw '录像仍在保存，请等待完成。'
        }
    }
    if ($CheckOnly) { Write-Host '只读检查完成；未编译、烧录或重启驾驶台。'; return }
    if (!$SafetyConfirmed) {
        Write-Host '烧录会复位 STM32。确认任务与录像已停止、DISARM、推进器断电或机体可靠固定。'
        if ((Read-Host '确认后输入 YES；其他输入取消') -cne 'YES') { throw '已取消，未烧录。' }
    }
    & (Join-Path $PSScriptRoot 'Maintain-Rov.ps1') -Action build -Keil $Keil -Rebuild
    # Stop only the console owning the requested local port, after build succeeds.
    if ($taskState) {
        $taskListener = Get-NetTCPConnection -State Listen -LocalPort $WebPort | Select-Object -First 1
        $taskProcess = Get-CimInstance Win32_Process -Filter "ProcessId=$($taskListener.OwningProcess)"
        if ($taskProcess.CommandLine -notmatch 'trial_control_web\.py') { throw '端口被其他程序占用，未烧录。' }
        Stop-Process -Id $taskProcess.ProcessId
    }
    $taskLogDir = Join-Path $taskRoot ('build/one-click/' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
    & $Python tools/auv/one_click_pi.py prepare --host $PiHost --user $PiUser --device /dev/serial0 --log-dir $taskLogDir
    if ($LASTEXITCODE -ne 0) { throw "Pi 配置或 MCU DISARM/中立确认失败，未烧录；日志：$taskLogDir" }
    & $Python tools/rov/maintenance.py flash --probe $Probe --pack $Pack
    if ($LASTEXITCODE -ne 0) { throw '烧录或读回校验失败，未启动驾驶台；请检查 build/maintenance 日志。' }
    & $Python tools/auv/one_click_pi.py finish --host $PiHost --user $PiUser --device /dev/serial0 --log-dir $taskLogDir
    if ($LASTEXITCODE -ne 0) { throw "固件已烧录，但 Pi 服务恢复失败；日志：$taskLogDir" }
    & $Python tools/rov/maintenance.py check --host $PiHost --user $PiUser
    if ($LASTEXITCODE -ne 0) { throw '固件已烧录，但 Pi 服务健康检查失败；请检查 build/maintenance 日志。' }
    & (Join-Path $PSScriptRoot 'Start-RovTest.ps1') -Python $Python -PiHost $PiHost -WebPort $WebPort -NoBrowser
    if (!$NoBrowser) { Start-Process "$taskUrl/" }
    Write-Host "独立 ROV 已烧录，Pi 服务检查通过，驾驶台：$taskUrl/"
    Write-Host '手柄未连接时驾驶台继续等待，不将其误报为烧录失败。'
    Write-Host '诊断日志已开启；未自动 ARM、水平校准或录像。重启后先岸上放平并显式水平校准。'
} finally {
    $env:PATH = $taskOldPath
    $env:PYTHONPATH = $taskOldPythonPath
    $env:AUV_DEPLOY_PASSWORD = $taskOldPassword
    Pop-Location
}
