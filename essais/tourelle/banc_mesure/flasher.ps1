# Compile un croquis de essais/tourelle et le téléverse sur l'ESP32 (COM3).
# Usage : flasher.ps1 nom_du_croquis
param([string]$croquis)
$cli = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$cfg = "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml"
$build = "$PSScriptRoot\build_$croquis"
$source = "c:\Users\kenfa\Documents\Projets\Robot+Lidar\essais\tourelle\$croquis"
& $cli compile --config-file $cfg --fqbn esp32:esp32:esp32 --warnings default --build-path $build $source | Select-Object -Last 3
if ($LASTEXITCODE -ne 0) { "ECHEC de la compilation"; exit 1 }
& $cli upload --config-file $cfg -p COM3 --fqbn esp32:esp32:esp32 --input-dir $build | Select-Object -Last 3
if ($LASTEXITCODE -ne 0) { "ECHEC du televersement"; exit 1 }
"Televersement reussi"
