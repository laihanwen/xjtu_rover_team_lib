@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Tag-Test.ps1" -Action start %*
pause
