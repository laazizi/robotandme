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

# Redirection via cmd : en PS 5.1, rediriger le stderr d'un exe natif fabrique
# des NativeCommandError (fatals avec EAP=Stop) meme pour un simple WARNING docker.
cmd /c "docker info >nul 2>&1"
if ($LASTEXITCODE -ne 0) {
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

# Transport RMW de libmicroros : le colcon.meta du composant est fige sur UDP,
# il faut le surcharger via app-colcon.meta (lu par le composant a la racine du
# projet) : serie -> custom (UART), eth -> udp. libmicroros est construit DANS
# le dossier du composant, donc idf.py fullclean ne suffit pas : tout changement
# de ce fichier impose de supprimer ses artefacts pour forcer sa reconstruction.
$RmwTransport = if ($Transport -eq "eth") { "udp" } else { "custom" }
$MetaFile = Join-Path $Root "app-colcon.meta"
$MetaContent = @"
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_TRANSPORT=$RmwTransport"
            ]
        }
    }
}
"@
$OldMeta = if (Test-Path $MetaFile) { [System.IO.File]::ReadAllText($MetaFile) } else { $null }
# Etat incoherent (build interrompu) : artefacts partiels -> tout rebuilder.
# include/rcl sert de temoin : absent = generation des headers interrompue.
$MicroRosStale = -not ((Test-Path (Join-Path $Component "libmicroros.a")) -and
                       (Test-Path (Join-Path $Component "include\rcl")))
if ($OldMeta -ne $MetaContent -or $MicroRosStale) {
    Write-Host ">> Transport RMW = $RmwTransport : reconstruction de libmicroros (long)..." -ForegroundColor Yellow
    [System.IO.File]::WriteAllText($MetaFile, $MetaContent)
    # Suppression DANS le conteneur : les chemins de micro_ros_src depassent
    # MAX_PATH (260 car.), Remove-Item echoue cote Windows.
    $SwitchCmd = "rm -rf components/micro_ros_espidf_component/{libmicroros.a,include,esp32_toolchain.cmake,micro_ros_src} && idf.py fullclean && "
}

# Deps python du build micro-ROS installees a chaque run (simple et sans etat) ;
# set-target uniquement si sdkconfig absent, pour ne pas ecraser les choix menuconfig.
$Setup = "pip3 install -q catkin_pkg lark-parser colcon-common-extensions 'empy==3.3.4'"
$Target = "if [ ! -f sdkconfig ]; then SDKCONFIG_DEFAULTS='$Defaults' idf.py set-target esp32p4; fi"

if ($Menuconfig) {
    docker run -it --rm -v "${Root}:/project" -w /project $IdfImage `
        bash -c "$Setup && $Target && idf.py menuconfig"
    exit $LASTEXITCODE
}

$Build = if ($Clean) { "${SwitchCmd}idf.py fullclean && $Target && idf.py build" }
         else { "$SwitchCmd$Target && idf.py build" }

Write-Host ">> Compilation (esp32p4, transport $Transport)..." -ForegroundColor Cyan
docker run --rm -v "${Root}:/project" -w /project $IdfImage `
    bash -c "$Setup && $Build"

if ($LASTEXITCODE -eq 0) {
    Write-Host ">> Build OK. Flasher avec : .\scripts\flash.ps1 -Port COM5" -ForegroundColor Green
} else {
    Write-Error "Echec de compilation (code $LASTEXITCODE)."
}
