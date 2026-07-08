# Flashe le firmware depuis Windows, directement sur le port COM (esptool).
# Pas besoin d'IDF ni de Docker : juste Python + 'pip install esptool'.
#
# Usage :
#   .\scripts\flash.ps1                    # auto-detecte le port COM s'il n'y en a qu'un
#   .\scripts\flash.ps1 -Port COM5
#   .\scripts\flash.ps1 -Port COM5 -Monitor   # ouvre le moniteur serie apres le flash

param(
    [string]$Port,
    [int]$Baud = 460800,
    [switch]$Monitor
)

$ErrorActionPreference = "Stop"
$Root = Split-Path $PSScriptRoot -Parent
$BuildDir = Join-Path $Root "build"

if (-not (Test-Path (Join-Path $BuildDir "flash_args"))) {
    Write-Error "Pas de build trouve ($BuildDir\flash_args manquant). Lancer .\scripts\build.ps1 d'abord."
}

python -m esptool version *> $null
if (-not $?) {
    Write-Error "esptool introuvable. Installer avec : pip install esptool"
}

if (-not $Port) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 1) {
        $Port = $ports[0]
        Write-Host ">> Port detecte : $Port" -ForegroundColor Cyan
    } elseif ($ports.Count -eq 0) {
        Write-Error "Aucun port COM detecte. Brancher la carte (port USB 'UART' de la carte EV)."
    } else {
        Write-Error "Plusieurs ports COM : $($ports -join ', '). Preciser avec -Port COMx"
    }
}

Write-Host ">> Flash sur $Port a $Baud bauds..." -ForegroundColor Cyan
Push-Location $BuildDir
try {
    # flash_args (genere par idf.py build) contient offsets + binaires, chemins relatifs au build dir
    python -m esptool --chip esp32p4 --port $Port --baud $Baud `
        --before default_reset --after hard_reset write_flash "@flash_args"
    $flashOk = ($LASTEXITCODE -eq 0)
} finally {
    Pop-Location
}

if (-not $flashOk) {
    Write-Error "Echec du flash (code $LASTEXITCODE). Essayer -Baud 115200, ou maintenir BOOT au reset."
}
Write-Host ">> Flash OK." -ForegroundColor Green

if ($Monitor) {
    & (Join-Path $PSScriptRoot "monitor.ps1") -Port $Port
}
