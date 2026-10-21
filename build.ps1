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

Write-Host "==> Building Bevy viewer from $root"

Push-Location $root
try {
    & $cargo.Source build -p cad-app
    $code = $LASTEXITCODE
    if ($code -ne 0) {
        throw "cargo build failed with exit code $code"
    }
    $exe = Join-Path $root "build\cargo\debug\cad-app.exe"
    Write-Host "==> Built $exe"
}
finally {
    Pop-Location
}
