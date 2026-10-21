# Main Mission (Sim) on a Windows laptop, in one go -- no drone, no radio:
#   1. opens a new window running the simulator in WSL (scripts/sim/start_sim.sh)
#   2. starts the GCS here with its "radio" on the simulator's TCP link
# The browser panel opens at http://127.0.0.1:8000/ui/ -- wait for SIM READY in
# the sim window (~4 min: ArduCopter needs a home position), then choose
# "Main Mission (Sim)" and press START. The Gazebo 3D panel is at the top.
#
# One-time setup (WSL Ubuntu 24.04): wsl -d Ubuntu-24.04 -- /mnt/d/NidarFiles/scripts/sim/setup_sim.sh
#
#   powershell -ExecutionPolicy Bypass -File .\scripts\sim\start_sim_windows.ps1
#   powershell -ExecutionPolicy Bypass -File .\scripts\sim\start_sim_windows.ps1 -Gui   # also Gazebo's own window
param(
    [switch]$Gui,
    [string]$Distro = "Ubuntu-24.04"
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
# D:\NidarFiles -> /mnt/d/NidarFiles
$wslRoot = "/mnt/" + $root.Substring(0, 1).ToLower() + ($root.Substring(2) -replace "\\", "/")
$simArgs = if ($Gui) { " --gui" } else { "" }

Write-Host "Starting the simulator in WSL ($Distro) in a new window..."
Start-Process -FilePath "wsl.exe" -ArgumentList @(
    "-d", $Distro, "--", "bash", "-lc",
    "`"$wslRoot/scripts/sim/start_sim.sh$simArgs; echo; read -p 'Simulator stopped -- press Enter to close'`""
)

Write-Host "Starting the GCS with the radio on the simulator's TCP link..."
& (Join-Path $PSScriptRoot "..\gcs\start_gcs.ps1") -RadioPort "socket://127.0.0.1:5800"
