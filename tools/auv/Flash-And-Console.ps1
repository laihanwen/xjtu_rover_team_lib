param(
 [string]$PiHost='192.168.137.150', [string]$PiUser='pi',
 [string]$Device='/dev/serial0', [string]$Probe='ATK 20190528',
 [string]$Keil='C:/Keil_v5/UV4/UV4.exe',
 [string]$Pack='C:/Keil_v5/ARM/PACK/Keil/STM32F4xx_DFP/1.0.8',
 [string]$Python='', [int]$WebPort=8767,
 [switch]$SafeToFlash, [switch]$CheckOnly
)
$ErrorActionPreference='Stop'
if ($Device -ne '/dev/serial0') { throw '此兼容入口不改动 Pi 串口，请先核实服务配置。' }
Write-Host '旧入口转到 tools/rov/Flash-And-Start-Rov.ps1，烧录独立 ROV 固件。'
& (Join-Path $PSScriptRoot '../rov/Flash-And-Start-Rov.ps1') -PiHost $PiHost -PiUser $PiUser -Probe $Probe -Keil $Keil -Pack $Pack -Python $Python -WebPort $WebPort -SafetyConfirmed:$SafeToFlash -CheckOnly:$CheckOnly
