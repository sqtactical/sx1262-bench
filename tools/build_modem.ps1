# Build the SX1262 modem with arduino-cli and convert it to UF2.
#   .\build_modem.ps1 [-Tools <dir with arduino-cli.exe and a Python venv>] [-Out <build dir>]
# Needs the Adafruit nRF52 core 1.7.0 (arduino-cli core install adafruit:nrf52).
param(
    [string]$Tools = "$env:LOCALAPPDATA\flrc-tools",
    [string]$Out = "$env:LOCALAPPDATA\flrc-tools\build1262"
)
$sketch = Join-Path $PSScriptRoot "..\firmware\sx1262_modem"
$core = "$env:LOCALAPPDATA\Arduino15\packages\adafruit\hardware\nrf52\1.7.0"
& "$Tools\arduino-cli.exe" compile -b adafruit:nrf52:pca10056 --output-dir $Out $sketch
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& "$Tools\venv\Scripts\python.exe" "$core\tools\uf2conv\uf2conv.py" -f 0xADA52840 -c -o "$Out\sx1262_modem.uf2" "$Out\sx1262_modem.ino.hex"
