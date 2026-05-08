param(
  [Parameter(Mandatory = $true)]
  [string]$WorkspaceFolder
)

$port = python "$WorkspaceFolder/PropaneScale/list_ports.py"
if (-not $port) {
  Write-Error "No COM port returned from list_ports.py"
  exit 1
}

$esptool = Join-Path $env:LOCALAPPDATA "Arduino15/packages/esp32/tools/esptool_py/5.1.0/esptool.exe"
$mergedBin = "$WorkspaceFolder/PropaneScale/build/esp32.esp32.esp32thing/PropaneScale.ino.merged.bin"

& $esptool --chip esp32 --port $port --baud 921600 write-flash -z 0x0 $mergedBin
