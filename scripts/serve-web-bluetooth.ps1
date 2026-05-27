param(
  [int]$Port = 8000,
  [string]$Folder = "",
  [string]$WorkspaceFolder = ""
)

# Prefer an explicit workspace folder path when called from VS Code tasks
if ($WorkspaceFolder -and (Test-Path $WorkspaceFolder)) {
  # prefer several possible locations that may exist in this repo
  $cand1 = Join-Path -Path $WorkspaceFolder -ChildPath "Documentation\web-bluetooth-client"
  $cand2 = Join-Path -Path $WorkspaceFolder -ChildPath "Documentation\Copilot\web-bluetooth-client"
  if (Test-Path $cand1) { $defaultFolder = $cand1 }
  elseif (Test-Path $cand2) { $defaultFolder = $cand2 }
  else { $defaultFolder = $cand1 }
} else {
  # fall back to path relative to the script location; try both common locations
  $cand1 = Join-Path -Path $PSScriptRoot -ChildPath "..\Documentation\web-bluetooth-client"
  $cand2 = Join-Path -Path $PSScriptRoot -ChildPath "..\Documentation\Copilot\web-bluetooth-client"
  if (Test-Path $cand1) { $defaultFolder = $cand1 }
  elseif (Test-Path $cand2) { $defaultFolder = $cand2 }
  else { $defaultFolder = $cand1 }
}

if (-not [string]::IsNullOrWhiteSpace($Folder)) {
  $serveFolder = $Folder
} else {
  $serveFolder = $defaultFolder
}

$resolved = Resolve-Path -Path $serveFolder -ErrorAction SilentlyContinue
if (-not $resolved) {
  Write-Error "Serve folder not found: $serveFolder"
  exit 1
}

$Folder = $resolved.Path
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
