param(
    [ValidateSet('AUV_A0','AUV_TAG_DOCK','AUV_ROV_DUAL')][string]$Profile = 'AUV_TAG_DOCK',
    [string]$Probe = 'ATK 20190528',
    [string]$Pack = 'C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8'
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskHex = Join-Path $taskRoot "firmware/stm32/MDK-ARM/$Profile/$Profile.hex"
if (!(Test-Path -LiteralPath $taskHex)) { throw "先构建独立 AUV 固件：$taskHex" }
$taskOldPythonPath = $env:PYTHONPATH
Push-Location $taskRoot
try {
    $taskPaths = @(Join-Path $taskRoot 'build/debug-deps')
    $env:PYTHONPATH = ($taskPaths + @($taskOldPythonPath) | Where-Object { $_ }) -join [IO.Path]::PathSeparator
    python tools/rov/maintenance.py flash --hex $taskHex --probe $Probe --pack $Pack
    if ($LASTEXITCODE -ne 0) { throw "AUV 烧录校验失败 ($LASTEXITCODE)" }
} finally {
    $env:PYTHONPATH = $taskOldPythonPath
    Pop-Location
}
