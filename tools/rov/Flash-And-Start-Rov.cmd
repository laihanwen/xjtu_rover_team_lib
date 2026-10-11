@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Flash-And-Start-Rov.ps1" %*
if errorlevel 1 pause
