<#
Starts the GCS on the Windows laptop. Everything goes over the MicroLR900
radio (default COM5 @ 115200): START/ABORT out, telemetry back from the
Jetson. There is no Wi-Fi link to the drone.
Then opens the operator panel at http://127.0.0.1:8000/ui/

Usage:
    .\scripts\gcs\start_gcs.ps1
    .\scripts\gcs\start_gcs.ps1 -RadioPort COM7
    .\scripts\gcs\start_gcs.ps1 -Setup      # bench only: adds the Pixhawk
                                             # parameter Setup section
                                             # (the Jetson needs --setup to write)

First run creates gcs\backend\.venv and installs the backend requirements.
The frontend is (re)built when gcs\frontend\dist is missing or older than
its sources (needs Node.js).
#>
param(
    [string]$RadioPort = "COM5",
    [int]$RadioBaud = 115200,
    [switch]$Setup
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$backend = Join-Path $root "custom-gcs\gcs\backend"
$frontend = Join-Path $root "custom-gcs\gcs\frontend"

# -- checks (warnings only: the GCS still starts and shows what is missing) --
$ports = [System.IO.Ports.SerialPort]::GetPortNames()
if ($RadioPort -like "socket://*") {
    Write-Host "SIMULATOR: radio link over TCP $RadioPort (scripts/sim/start_sim.sh must be running)"
} elseif ($ports -contains $RadioPort) {
    Write-Host "OK: radio port $RadioPort present"
} else {
    Write-Warning "radio port $RadioPort not found (present: $($ports -join ', ')). No telemetry and no START/ABORT until the radio is plugged in -- check Device Manager > Ports."
}

# -- backend environment ------------------------------------------------------
$venvPython = Join-Path $backend ".venv\Scripts\python.exe"
if (-not (Test-Path $venvPython)) {
    Write-Host "Creating backend virtualenv..."
    python -m venv (Join-Path $backend ".venv")
    & $venvPython -m pip install -r (Join-Path $backend "requirements.txt")
}

# -- frontend build -------------------------------------------------------------
$built = Join-Path $frontend "dist\index.html"
$stale = -not (Test-Path $built)
if (-not $stale) {
    $builtAt = (Get-Item $built).LastWriteTime
    $stale = [bool](Get-ChildItem (Join-Path $frontend "src") -Recurse -File |
        Where-Object { $_.LastWriteTime -gt $builtAt } | Select-Object -First 1)
}
if ($stale) {
    Write-Host "Building frontend..."
    Push-Location $frontend
    try {
        if (-not (Test-Path (Join-Path $frontend "node_modules"))) { npm install }
        npm run build
    } finally {
        Pop-Location
    }
}

# -- run ----------------------------------------------------------------------
$env:GCS_TELEMETRY_SOURCE = "radio"
$env:GCS_ROS_ENABLED = "false"
$env:GCS_RADIO_PORT = $RadioPort
$env:GCS_RADIO_BAUD = "$RadioBaud"
$env:GCS_RADIO_ENABLED = "true"
$env:GCS_SETUP_ENABLED = if ($Setup) { "true" } else { "false" }
if ($Setup) {
    Write-Warning "BENCH SETUP MODE: the panel can read/write Pixhawk parameters. Do not use this mode for a mission."
}

Write-Host "Starting GCS: radio $RadioPort @ $RadioBaud (commands + telemetry)"
Start-Process "http://127.0.0.1:8000/ui/"
Push-Location $backend
try {
    & $venvPython -m uvicorn app.main:app --host 127.0.0.1 --port 8000
} finally {
    Pop-Location
}
