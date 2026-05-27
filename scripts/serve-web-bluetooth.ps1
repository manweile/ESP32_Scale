param(
  [int]$Port = 8000,
  [string]$Folder = "$PSScriptRoot/..\Documentation\Copilot\web-bluetooth-client"
)

Write-Host "Serving web-bluetooth-client from: $Folder"
Write-Host "Open http://localhost:$Port in Chrome (localhost is treated as a secure context)."

# Find python or python3 robustly
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) { $py = Get-Command python3 -ErrorAction SilentlyContinue }
if (-not $py) {
  Write-Error "Python not found in PATH. Install Python 3 or run an alternative HTTP server."
  exit 1
}

# Prefer the full path to the executable if available
$pyExe = $py.Path
if (-not $pyExe) { $pyExe = $py.Name }

Push-Location $Folder
try {
  & $pyExe -m http.server $Port
} finally {
  Pop-Location
}
