param(
    [ValidateSet('AUV_A0','AUV_TAG_DOCK')][string]$Profile = 'AUV_TAG_DOCK',
    [string]$Probe = 'ATK 20190528',
    [string]$Pack = 'C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8',
    [string]$Python = ''
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskHex = Join-Path $taskRoot "firmware/stm32/MDK-ARM/$Profile/$Profile.hex"
if (!(Test-Path -LiteralPath $taskHex)) { throw "先构建独立 AUV 固件：$taskHex" }
if (!$Python) {
    $taskCandidates = @(
        "$env:LOCALAPPDATA/xjtu-auv/console-venv/Scripts/python.exe",
        "$env:USERPROFILE/AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/xjtu-auv/console-venv/Scripts/python.exe"
    )
    $Python = $taskCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if (!$Python) { $Python = (Get-Command python -ErrorAction Stop).Source }
}
$taskOldPythonPath = $env:PYTHONPATH
Push-Location $taskRoot
try {
    $taskPaths = @(Join-Path $taskRoot 'build/debug-deps')
    $env:PYTHONPATH = ($taskPaths + @($taskOldPythonPath) | Where-Object { $_ }) -join [IO.Path]::PathSeparator
    & $Python -c 'import intelhex, pyocd'
    if ($LASTEXITCODE -ne 0) { throw '烧录依赖缺失；请用 -Python 指定已有 intelhex/pyocd 的项目环境，未操作调试器。' }
    & $Python tools/rov/maintenance.py flash --hex $taskHex --probe $Probe --pack $Pack
    if ($LASTEXITCODE -ne 0) { throw "AUV 烧录校验失败 ($LASTEXITCODE)" }
} finally {
    $env:PYTHONPATH = $taskOldPythonPath
    Pop-Location
}
