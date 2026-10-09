@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Flash-And-Console.ps1" %*
if errorlevel 1 pause
