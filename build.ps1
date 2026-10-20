#Requires -Version 7.0
[CmdletBinding()]
param()

# One-click build: Cargo only, all artifacts under .\build.
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$env:RUSTUP_TOOLCHAIN = "1.99.1-x86_64-pc-windows-gnu"

$cargo = Get-Command cargo -ErrorAction SilentlyContinue
if (-not $cargo) {
    throw "cargo was not found in PATH"
}

Push-Location $root
try {
    & $cargo.Source build -p cad-app
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
