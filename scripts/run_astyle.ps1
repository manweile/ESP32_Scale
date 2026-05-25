$files = Get-ChildItem -Path 'D:\MyArduino\Projects\ESP32_Scale\PropaneScale' -Recurse -Include *.c,*.cpp,*.h,*.hpp,*.ino
foreach ($f in $files) {
    Write-Output "Formatting: $($f.FullName)"
    & 'C:\AStyle\astyle.exe' --options='D:\MyArduino\Projects\ESP32_Scale\.astylerc' $f.FullName
}