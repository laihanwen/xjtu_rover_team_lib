param(
    [ValidateSet('check','start','task')][string]$Action = 'check',
    [string]$Python = '',
    [switch]$NoBrowser,
    [switch]$LocalOnly
)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$env:PYTHONIOENCODING = 'utf-8'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskRoots = @(
    (Join-Path $env:LOCALAPPDATA 'xjtu-auv'),
    (Join-Path $env:USERPROFILE 'AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/xjtu-auv')
) | Select-Object -Unique
if (!$Python) {
    $Python = $taskRoots | ForEach-Object { Join-Path $_ 'console-venv/Scripts/python.exe' } |
        Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (!$Python -or !(Test-Path -LiteralPath $Python)) { throw '未找到项目工具虚拟环境，请用 -Python 指定安装了 paramiko 的 Python。' }
$taskOldPassword = $env:AUV_DEPLOY_PASSWORD
Push-Location $taskRoot
try {
    & $Python -c 'import paramiko'
    if ($LASTEXITCODE -ne 0) { throw '所选 Python 缺少 paramiko，请使用项目工具虚拟环境。' }
    if (!$env:AUV_DEPLOY_PASSWORD) {
        $taskFile = $taskRoots | ForEach-Object { Join-Path $_ 'pi-password.dpapi' } |
            Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
        if (!$taskFile) { throw '未找到已保存的 Pi 凭据。' }
        if (!(Test-Path -LiteralPath $taskFile)) { throw '未找到已保存的 Pi 凭据。' }
        $taskSecret = (Get-Content -LiteralPath $taskFile -Raw).Trim() | ConvertTo-SecureString
        $env:AUV_DEPLOY_PASSWORD = ([System.Net.NetworkCredential]::new('', $taskSecret)).Password
    }
    if ($LocalOnly) {
        Write-Output "本地 Python、paramiko 和凭据读取检查通过：$Python；未连接设备。"
        return
    }
    $taskLogs = Join-Path $taskRoot ('build/auv-entry/' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
    & $Python tools/auv/tag_task_entry.py $Action --log-dir $taskLogs
    if ($LASTEXITCODE -ne 0) { throw "检查或启动未通过，日志：$taskLogs" }
    if ($Action -eq 'start' -and !$NoBrowser) { Start-Process 'http://192.168.137.150:8080/' }
    Write-Output "日志：$taskLogs；此入口未执行 ARM。"
} finally {
    $env:AUV_DEPLOY_PASSWORD = $taskOldPassword
    Pop-Location
}
