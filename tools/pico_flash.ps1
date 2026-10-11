# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# pico_flash.ps1 - Flash a running Pico (W) from Windows without touching the
# BOOTSEL button, then show its console.
#
#   powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_gamepad.uf2 [-ReadMs 8000]
#
# How: opening the board's USB serial port at 1200 baud asks the Pico SDK's
# stdio_usb to reboot into the ROM bootloader, which mounts the RPI-RP2 (Pico 2: RP2350) drive;
# copying a .uf2 there flashes and restarts the board. (picotool needs a WinUSB
# driver on the reset interface, which a stock Windows install lacks.)
param([Parameter(Mandatory = $true)][string]$Uf2, [int]$ReadMs = 8000)

function Find-PicoPort { (Get-CimInstance Win32_SerialPort | Where-Object PNPDeviceID -match 'VID_2E8A').DeviceID }

$port = Find-PicoPort
if ($port) { try { $p = New-Object System.IO.Ports.SerialPort $port, 1200; $p.Open(); $p.Close() } catch {} }

$drive = $null
for ($i = 0; $i -lt 40 -and -not $drive; $i++) {
    Start-Sleep -Milliseconds 250
    $drive = (Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' -or $_.FileSystemLabel -eq 'RP2350' }).DriveLetter
}
if (-not $drive) { Write-Error "No RPI-RP2 / RP2350 drive: hold BOOTSEL while plugging the board in, then retry."; exit 1 }
Copy-Item $Uf2 "${drive}:\"
Write-Host "flashed $Uf2 via ${drive}:"

if ($ReadMs -le 0) { exit 0 }
Start-Sleep -Milliseconds 300
$port = $null
for ($i = 0; $i -lt 50 -and -not $port; $i++) { $port = Find-PicoPort; if (-not $port) { Start-Sleep -Milliseconds 100 } }
if (-not $port) { Write-Error "board did not re-enumerate as a serial port"; exit 1 }
$p = New-Object System.IO.Ports.SerialPort $port, 115200
$p.DtrEnable = $true
$p.Open()
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.ElapsedMilliseconds -lt $ReadMs) {
    $s = $p.ReadExisting(); if ($s) { [Console]::Out.Write($s) }
    Start-Sleep -Milliseconds 50
}
$p.Close()
