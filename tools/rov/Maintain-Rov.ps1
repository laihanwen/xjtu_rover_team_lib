param(
    [ValidateSet('build','flash','deploy','check','all')][string]$Action = 'check',
    [string]$Keil = 'C:/Keil_v5/UV4/UV4.exe',
    [string]$Probe = 'ATK 20190528',
    [string]$Pack = 'C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8',
    [string]$PiHost = '192.168.137.150',
    [string]$PiUser = 'pi',
    [switch]$ApplyPiProfile
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskOldDeployPassword = $env:AUV_DEPLOY_PASSWORD
Push-Location $taskRoot
try {
    # DPAPI credential belongs to the current Windows user; never commit its value.
    $taskCredentialFile = Join-Path $env:LOCALAPPDATA 'xjtu-auv/pi-password.dpapi'
    if ($Action -in @('deploy','check','all') -and !$env:AUV_DEPLOY_PASSWORD -and
        $PiHost -eq '192.168.137.150' -and $PiUser -eq 'pi' -and
        (Test-Path -LiteralPath $taskCredentialFile)) {
        $taskSecret = (Get-Content -LiteralPath $taskCredentialFile -Raw).Trim() | ConvertTo-SecureString
        $env:AUV_DEPLOY_PASSWORD = ([System.Net.NetworkCredential]::new('', $taskSecret)).Password
    }
    $taskPaths = @('build/debug-deps','build/deploy-ssh') | ForEach-Object { Join-Path $taskRoot $_ }
    $taskOldPythonPath = $env:PYTHONPATH
    $env:PYTHONPATH = ($taskPaths + @($taskOldPythonPath) | Where-Object { $_ }) -join [IO.Path]::PathSeparator
    if ($Action -in @('build','all')) {
        if (!(Test-Path -LiteralPath $Keil)) { throw "Keil not found: $Keil" }
        New-Item -ItemType Directory -Force build/maintenance | Out-Null
        $taskLog = Join-Path $taskRoot ('build/maintenance/keil-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.log')
        $taskProject = Join-Path $taskRoot 'firmware/stm32/MDK-ARM/Copy_cup.uvprojx'
        $taskProcess = Start-Process -FilePath $Keil -ArgumentList @('-b',('"' + $taskProject + '"'),'-o',('"' + $taskLog + '"')) -WindowStyle Hidden -Wait -PassThru
        $taskOutput = Get-Content -LiteralPath $taskLog -Raw
        Write-Output ($taskOutput -split "`n" | Select-Object -Last 12)
        if ($taskProcess.ExitCode -ne 0 -or $taskOutput -notmatch '0 Error\(s\)') { throw "Build failed; see $taskLog" }
    }
    if ($Action -ne 'build') {
        $taskArgs = @($Action,'--probe',$Probe,'--pack',$Pack,'--host',$PiHost,'--user',$PiUser)
        if ($ApplyPiProfile) { $taskArgs += '--apply-pi-profile' }
        python tools/rov/maintenance.py @taskArgs
        if ($LASTEXITCODE -ne 0) { throw "Maintenance failed ($LASTEXITCODE)" }
    }
} finally {
    $env:AUV_DEPLOY_PASSWORD = $taskOldDeployPassword
    $env:PYTHONPATH = $taskOldPythonPath
    Pop-Location
}
