#Requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $AppArguments
)

# One-click run: build on demand, then launch the Bevy viewer.
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$exe = Join-Path $root "build\cargo\debug\cad-app.exe"

if (-not (Test-Path -LiteralPath $exe)) {
    & (Join-Path $root "build.ps1")
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

& $exe @AppArguments
exit $LASTEXITCODE
