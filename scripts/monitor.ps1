# ============================================================================
# NON ADAPTE au decoupage en controleurs (controllers/, septembre 2026).
# Ce script suppose le projet ESP-IDF A LA RACINE (sdkconfig, main/, build/),
# disposition qui n'existe plus. Sous Linux / WSL2, utiliser :
#     ./scripts/build.sh <controleur> [build|clean|menuconfig] [serial|eth]
#     ./scripts/flash.sh <controleur> [/dev/ttyACM0] [monitor]
# A reprendre si le build Windows redevient necessaire.
# ============================================================================
# Moniteur serie simple (logs du firmware a 115200 bauds).
# Requiert pyserial : pip install pyserial
# Quitter : Ctrl+]
#
# NB : UART0 est partage entre les logs et le transport micro-ROS serie —
# fermer le moniteur avant de lancer le micro-ros-agent sur ce port.

param(
    [string]$Port,
    [int]$Baud = 115200
)

$ErrorActionPreference = "Stop"

if (-not $Port) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 1) { $Port = $ports[0] }
    else { Write-Error "Preciser le port : .\scripts\monitor.ps1 -Port COM5 (detectes : $($ports -join ', '))" }
}

Write-Host ">> Moniteur sur $Port a $Baud bauds (Ctrl+] pour quitter)" -ForegroundColor Cyan
python -m serial.tools.miniterm $Port $Baud
