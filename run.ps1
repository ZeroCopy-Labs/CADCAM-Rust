#Requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $AppArguments
)

# One-click run: always rebuild first so a stale executable is never launched.
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$exe = Join-Path $root "build\cargo\debug\cad-app.exe"

& (Join-Path $root "build.ps1")
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

if (-not (Test-Path -LiteralPath $exe)) {
    throw "Build succeeded but $exe was not found"
}

Write-Host "==> Running $exe"
& $exe @AppArguments
exit $LASTEXITCODE
