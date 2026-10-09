param(
    [string]$Keil = 'C:/Keil_v5/UV4/UV4.exe',
    [ValidateSet('AUV_A0','AUV_TAG_DOCK')][string]$Profile = 'AUV_A0',
    [switch]$Rebuild
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$taskMdk = Join-Path $taskRoot 'firmware/stm32/MDK-ARM'
$taskProject = Get-Content -LiteralPath (Join-Path $taskMdk 'Copy_cup.uvprojx') -Raw
$taskProject = $taskProject.Replace('<TargetName>Copy_cup</TargetName>',"<TargetName>$Profile</TargetName>")
$taskProject = $taskProject.Replace('<OutputDirectory>Copy_cup\</OutputDirectory>',"<OutputDirectory>$Profile\</OutputDirectory>")
$taskProject = $taskProject.Replace('<OutputName>Copy_cup</OutputName>',"<OutputName>$Profile</OutputName>")
$taskDefines = 'USE_HAL_DRIVER,STM32F405xx,AUV_AUTONOMOUS_PROFILE=1'
if ($Profile -eq 'AUV_TAG_DOCK') { $taskDefines += ',AUV_AUTONOMY_MAX_DEPTH_M=1.2f' }
$taskProject = $taskProject.Replace('<Define>USE_HAL_DRIVER,STM32F405xx</Define>',"<Define>$taskDefines</Define>")
$taskGenerated = Join-Path $taskMdk 'Copy_cup_auv.uvprojx'
$taskLog = Join-Path $taskRoot ("build/auv/keil-$($Profile.ToLowerInvariant()).log")
New-Item -ItemType Directory -Force (Split-Path $taskLog) | Out-Null
try {
    [IO.File]::WriteAllText($taskGenerated, $taskProject, [Text.UTF8Encoding]::new($false))
    $taskBuildAction=if($Rebuild){'-r'}else{'-b'}
    $taskProcess = Start-Process -FilePath $Keil -ArgumentList @($taskBuildAction,('"'+$taskGenerated+'"'),'-o',('"'+$taskLog+'"')) -WindowStyle Hidden -Wait -PassThru
    $taskOutput = Get-Content -LiteralPath $taskLog -Raw
    Write-Output ($taskOutput -split "`n" | Select-Object -Last 14)
    if ($taskProcess.ExitCode -gt 1 -or $taskOutput -notmatch '0 Error\(s\)') { throw "AUV build failed: $taskLog" }
    Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $taskMdk "$Profile/$Profile.hex")
} finally {
    if (Test-Path -LiteralPath $taskGenerated) { Remove-Item -LiteralPath $taskGenerated }
}
