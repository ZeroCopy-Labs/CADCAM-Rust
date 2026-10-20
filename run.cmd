REM One-click run wrapper; forwards arguments to run.ps1.
@echo off
pwsh -NoProfile -ExecutionPolicy Bypass -File "%~dp0run.ps1" %*