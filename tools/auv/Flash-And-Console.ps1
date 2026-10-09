param(
    [string]$PiHost='192.168.137.150',
    [string]$PiUser='pi',
    [string]$Device='/dev/serial0',
    [string]$Probe='ATK 20190528',
    [string]$Keil='C:/Keil_v5/UV4/UV4.exe',
    [string]$Pack='C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8',
    [string]$Python='',
    [int]$WebPort=8767,
    [switch]$SafeToFlash,
    [switch]$CheckOnly
)
$ErrorActionPreference='Stop'
$taskRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskOldPath=$env:PYTHONPATH
$taskOldPassword=$env:AUV_DEPLOY_PASSWORD
$taskLogDir=Join-Path $taskRoot ('build/one-click/'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
Push-Location $taskRoot
try {
    if(!$Python){
        $taskCandidates=@(
            (Join-Path $env:LOCALAPPDATA 'xjtu-auv/console-venv/Scripts/python.exe'),
            (Join-Path $env:USERPROFILE 'AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/xjtu-auv/console-venv/Scripts/python.exe')
        )
        $Python=$taskCandidates | Where-Object {Test-Path -LiteralPath $_} | Select-Object -First 1
        if(!$Python){$Python='python'}
    }
    $taskPython=(Get-Command $Python -ErrorAction Stop).Source
    if(!(Test-Path -LiteralPath $Keil)){throw '未找到 Keil 编译器'}
    if(!(Test-Path -LiteralPath (Join-Path $Pack 'Flash/STM32F4xx_1024.FLM'))){throw '未找到 STM32F405 烧录算法'}
    $taskDeps=@('build/debug-deps','build/deploy-ssh','build/joystick-deps') | ForEach-Object {Join-Path $taskRoot $_}
    $env:PYTHONPATH=($taskDeps+@($taskOldPath) | Where-Object {$_}) -join [IO.Path]::PathSeparator
    & $taskPython -c 'import paramiko,serial,pyocd,intelhex,pygame'
    if($LASTEXITCODE -ne 0){throw '缺少 Python 依赖；请按一键入口文档配置独立环境'}
    if($CheckOnly){Write-Host '本地依赖检查通过；未连接设备、编译、烧录或启动服务。';return}
    if(!$SafeToFlash -and (Read-Host '请确认已停止任务、推进器断电或机器人可靠固定。输入 SAFE_TO_FLASH 继续') -cne 'SAFE_TO_FLASH') {throw '未确认烧录环境，已取消'}
    $taskCredential=Join-Path $env:LOCALAPPDATA 'xjtu-auv/pi-password.dpapi'
    if(!$env:AUV_DEPLOY_PASSWORD -and $PiHost -eq '192.168.137.150' -and $PiUser -eq 'pi' -and (Test-Path -LiteralPath $taskCredential)){
        $taskSecret=(Get-Content -LiteralPath $taskCredential -Raw).Trim() | ConvertTo-SecureString
        $env:AUV_DEPLOY_PASSWORD=([Net.NetworkCredential]::new('',$taskSecret)).Password
    }
    New-Item -ItemType Directory -Force $taskLogDir | Out-Null
    # Rebuild from this working tree, not a downloaded or cached HEX.
    & (Join-Path $taskRoot 'tools/rov/Maintain-Rov.ps1') -Action build -Keil $Keil
    $taskHex=Join-Path $taskRoot 'firmware/stm32/MDK-ARM/Copy_cup/Copy_cup.hex'
    $taskHash=(Get-FileHash -Algorithm SHA256 -LiteralPath $taskHex).Hash
    Set-Content -LiteralPath (Join-Path $taskLogDir 'firmware-sha256.txt') -Value $taskHash
    # Close only this checkout's console. Never leave an old operator running across reset.
    $taskEntry=Join-Path $taskRoot 'tools/rov/trial_control_web.py'
    Get-CimInstance Win32_Process | Where-Object { $_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -like "*$taskEntry*" } | ForEach-Object {Stop-Process -Id $_.ProcessId -ErrorAction SilentlyContinue}
    $taskPortCheck=[Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback,$WebPort)
    try {$taskPortCheck.Start()} finally {$taskPortCheck.Stop()}
    & $taskPython tools/auv/one_click_pi.py prepare --host $PiHost --user $PiUser --device $Device --log-dir $taskLogDir
    if($LASTEXITCODE -ne 0){throw 'Pi 准备或中立确认失败；未执行烧录'}
    & $taskPython tools/rov/maintenance.py flash --hex $taskHex --probe $Probe --pack $Pack
    if($LASTEXITCODE -ne 0){throw '烧录/读回校验失败；保留 Pi 控制服务停止状态'}
    & $taskPython tools/auv/one_click_pi.py finish --host $PiHost --user $PiUser --device $Device --log-dir $taskLogDir
    if($LASTEXITCODE -ne 0){throw '独立 ROV 固件确认或 Pi 服务启动失败；未启动 PC 驾驶台'}
    & (Join-Path $taskRoot 'tools/rov/Start-RovTest.ps1') -PiHost $PiHost -WebPort $WebPort -NoBrowser -Python $taskPython
    Start-Process "http://127.0.0.1:$WebPort/"
    Write-Host "已烧录并读回校验：$taskHash；驾驶台保持等待显式 ARM。记录：$taskLogDir"
} finally {
    $env:PYTHONPATH=$taskOldPath;$env:AUV_DEPLOY_PASSWORD=$taskOldPassword
    Pop-Location
}
