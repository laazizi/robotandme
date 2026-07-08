# Compile le firmware dans Docker (image officielle ESP-IDF v5.5 + deps micro-ROS).
# Le composant micro-ROS ne se compile que sous Linux : Docker evite d'installer WSL2/IDF.
#
# Usage :
#   .\scripts\build.ps1                        # build (transport actuel, serie par defaut)
#   .\scripts\build.ps1 -Transport eth         # bascule sur Ethernet/UDP (voir sdkconfig.eth)
#   .\scripts\build.ps1 -Transport serial      # retour au transport serie
#   .\scripts\build.ps1 -Clean                 # fullclean + build
#   .\scripts\build.ps1 -Menuconfig            # ouvre menuconfig (agent IP, pins...)
#
# Premier lancement : long (telechargement image ~2 Go + build complet de libmicroros).
# Les suivants sont incrementaux. Changer de transport reconstruit libmicroros.

param(
    [ValidateSet("serial", "eth")]
    [string]$Transport,
    [switch]$Clean,
    [switch]$Menuconfig
)

$ErrorActionPreference = "Stop"
$Root = Split-Path $PSScriptRoot -Parent
$IdfImage = "espressif/idf:release-v5.5"
$Component = Join-Path $Root "components\micro_ros_espidf_component"
$Sdkconfig = Join-Path $Root "sdkconfig"

docker info *> $null
if (-not $?) {
    Write-Error "Docker n'est pas disponible. Lancer Docker Desktop d'abord."
}

if (-not (Test-Path $Component)) {
    Write-Host ">> Clonage du composant micro-ROS (branche humble)..." -ForegroundColor Cyan
    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git $Component
    if (-not $?) { Write-Error "Echec du clonage du composant micro-ROS." }
}

# Transport courant lu dans sdkconfig ; defaut = serie
$Current = "serial"
if ((Test-Path $Sdkconfig) -and (Select-String -Path $Sdkconfig -Pattern "^CONFIG_MICRO_ROS_ESP_NETIF_ENET=y" -Quiet)) {
    $Current = "eth"
}
if (-not $Transport) { $Transport = $Current }

# Changement de transport : regenerer sdkconfig et rebuilder libmicroros
$SwitchCmd = ""
if ($Transport -ne $Current -and (Test-Path $Sdkconfig)) {
    Write-Host ">> Changement de transport : $Current -> $Transport" -ForegroundColor Yellow
    Remove-Item $Sdkconfig -Force
    $SwitchCmd = "idf.py fullclean && "
}
$Defaults = "sdkconfig.defaults;sdkconfig.$Transport"

# Deps python du build micro-ROS installees a chaque run (simple et sans etat) ;
# set-target uniquement si sdkconfig absent, pour ne pas ecraser les choix menuconfig.
$Setup = "pip3 install -q catkin_pkg lark-parser colcon-common-extensions 'empy==3.3.4'"
$Target = "if [ ! -f sdkconfig ]; then SDKCONFIG_DEFAULTS='$Defaults' idf.py set-target esp32p4; fi"

if ($Menuconfig) {
    docker run -it --rm -v "${Root}:/project" -w /project $IdfImage `
        bash -c "$Setup && $Target && idf.py menuconfig"
    exit $LASTEXITCODE
}

$Build = if ($Clean) { "idf.py fullclean && $Target && idf.py build" }
         else { "$SwitchCmd$Target && idf.py build" }

Write-Host ">> Compilation (esp32p4, transport $Transport)..." -ForegroundColor Cyan
docker run --rm -v "${Root}:/project" -w /project $IdfImage `
    bash -c "$Setup && $Build"

if ($LASTEXITCODE -eq 0) {
    Write-Host ">> Build OK. Flasher avec : .\scripts\flash.ps1 -Port COM5" -ForegroundColor Green
} else {
    Write-Error "Echec de compilation (code $LASTEXITCODE)."
}
