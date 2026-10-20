REM One-click build wrapper; the real work is in build.ps1.
@echo off
pwsh -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*