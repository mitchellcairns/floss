# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# pico_flash.ps1 - Flash a running Pico (W) from Windows without touching the
# BOOTSEL button, then show its console.
#
#   powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_gamepad.uf2 [-ReadMs 8000]
#   powershell -File tools/pico_flash.ps1 -List
#   powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_link_dongle.uf2 -Serial E661640843406A29
#   powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_link_pad.uf2 -Port COM5
#
# How: opening the board's USB serial port at 1200 baud asks the firmware
# (the Pico SDK's stdio_usb, or the link dongle's own CDC) to reboot into the
# ROM bootloader, which mounts the RPI-RP2 (Pico 2: RP2350) drive; copying a
# .uf2 there flashes and restarts the board. (picotool needs a WinUSB driver
# on the reset interface, which a stock Windows install lacks.)
#
# Several boards: -List shows each board's COM port and USB serial number
# (the flash unique ID; the same for every FLOSS firmware on that board).
# -Serial <substring> or -Port COMx picks the board to flash; with more than
# one board connected one of them is required. The console is then read
# from the same board (found again by serial: the COM port number changes
# when the firmware's USB identity does, e.g. pad <-> dongle). If a second
# RPI-RP2 drive is present the script refuses rather than guess which board
# it belongs to. A board already in BOOTSEL mode has no serial port; it is
# flashed only when it is the one board connected.
param(
    [string]$Uf2,
    [int]$ReadMs = 8000,
    [string]$Serial,
    [string]$Port,
    [switch]$List
)

# USB identities: any Raspberry Pi VID (Pico SDK stdio: gamepad, LE gamepad,
# link pad) and the link dongle (pid.codes placeholder 1209:0F11).
$PicoIds = 'VID_2E8A|VID_1209&PID_0F11'
$DriveLabels = @('RPI-RP2', 'RP2350')

function Get-UsbSerial([string]$pnpId) {
    # A composite device's serial port is the child "USB\VID_x&PID_y&MI_00\6&...":
    # the USB serial number is the last part of its parent's instance ID,
    # "USB\VID_x&PID_y\<serial>". A non-composite port carries it itself.
    $id = $pnpId
    if ($pnpId -match '&MI_[0-9A-F]{2}') {
        try {
            $id = (Get-PnpDeviceProperty -InstanceId $pnpId -KeyName DEVPKEY_Device_Parent -ErrorAction Stop).Data
        } catch {
            return ''
        }
    }
    return ($id -split '\\')[-1]
}

function Get-PicoPorts {
    Get-CimInstance Win32_SerialPort | Where-Object { $_.PNPDeviceID -match $PicoIds } | ForEach-Object {
        $kind = if ($_.PNPDeviceID -match 'VID_1209&PID_0F11') { 'FLOSS Link Dongle' } else { 'Pico SDK USB stdio' }
        [pscustomobject]@{ Port = $_.DeviceID; Serial = (Get-UsbSerial $_.PNPDeviceID); Kind = $kind }
    } | Sort-Object Port
}

function Get-BootDrives {
    @(Get-Volume | Where-Object { $DriveLabels -contains $_.FileSystemLabel -and $_.DriveLetter } |
        ForEach-Object { [string]$_.DriveLetter })
}

if ($List) {
    $ports = @(Get-PicoPorts)
    if ($ports.Count -eq 0) { Write-Host 'no Pico serial port found' }
    foreach ($b in $ports) { Write-Host ("{0,-6} serial {1,-18} {2}" -f $b.Port, $b.Serial, $b.Kind) }
    foreach ($d in Get-BootDrives) { Write-Host "${d}:     a board in BOOTSEL mode (no serial port)" }
    exit 0
}
if (-not $Uf2) { Write-Error 'give -Uf2 <file> (or -List)'; exit 1 }
if (-not (Test-Path $Uf2)) { Write-Error "no such file: $Uf2"; exit 1 }

# ---- pick the board -----------------------------------------------------------
$ports = @(Get-PicoPorts)
$target = $null
if ($Port) {
    $target = $ports | Where-Object { $_.Port -ieq $Port } | Select-Object -First 1
    if (-not $target) { $target = [pscustomobject]@{ Port = $Port.ToUpper(); Serial = ''; Kind = '?' } }
} elseif ($Serial) {
    $match = @($ports | Where-Object { $_.Serial -and $_.Serial.IndexOf($Serial, [StringComparison]::OrdinalIgnoreCase) -ge 0 })
    if ($match.Count -eq 0) { Write-Error "no running board with serial '*$Serial*' (see -List)"; exit 1 }
    if ($match.Count -gt 1) { Write-Error "serial '*$Serial*' matches $($match.Count) boards: $(($match | ForEach-Object { $_.Serial }) -join ', ')"; exit 1 }
    $target = $match[0]
} elseif ($ports.Count -gt 1) {
    Write-Error "$($ports.Count) boards connected ($(($ports | ForEach-Object { "$($_.Port) $($_.Serial)" }) -join ', ')): choose one with -Serial or -Port (see -List)"
    exit 1
} elseif ($ports.Count -eq 1) {
    $target = $ports[0]
}

# ---- reboot it into the bootloader --------------------------------------------
$before = @(Get-BootDrives)
if ($target) {
    if ($before.Count -gt 0) {
        Write-Error "a board is already in BOOTSEL mode ($(($before | ForEach-Object { "${_}:" }) -join ' ')); flash or unplug it first - refusing to guess which drive is $($target.Port)"
        exit 1
    }
    Write-Host "rebooting $($target.Port) (serial $($target.Serial)) into BOOTSEL"
    try { $p = New-Object System.IO.Ports.SerialPort $target.Port, 1200; $p.Open(); $p.Close() } catch {}
}

$drive = $null
for ($i = 0; $i -lt 40 -and -not $drive; $i++) {
    Start-Sleep -Milliseconds 250
    $drives = @(Get-BootDrives)
    if ($drives.Count -gt 1) {
        Write-Error "$($drives.Count) RPI-RP2 / RP2350 drives ($(($drives | ForEach-Object { "${_}:" }) -join ' ')): refusing to guess which board to flash"
        exit 1
    }
    if ($drives.Count -eq 1) { $drive = $drives[0] }
}
if (-not $drive) { Write-Error "No RPI-RP2 / RP2350 drive: hold BOOTSEL while plugging the board in, then retry."; exit 1 }
Start-Sleep -Milliseconds 300   # a second board rebooting at the same moment would show up now
$drives = @(Get-BootDrives)
if ($drives.Count -gt 1) {
    Write-Error "$($drives.Count) RPI-RP2 / RP2350 drives ($(($drives | ForEach-Object { "${_}:" }) -join ' ')): refusing to guess which board to flash"
    exit 1
}
Copy-Item $Uf2 "${drive}:\"
Write-Host "flashed $Uf2 via ${drive}:"

# ---- show the console of the same board ---------------------------------------
if ($ReadMs -le 0) { exit 0 }
Start-Sleep -Milliseconds 300
$serialWanted = if ($target -and $target.Serial) { $target.Serial } else { '' }
$port = $null
for ($i = 0; $i -lt 50 -and -not $port; $i++) {
    $now = @(Get-PicoPorts)
    if ($serialWanted) {
        $port = ($now | Where-Object { $_.Serial -eq $serialWanted } | Select-Object -First 1).Port
    } elseif ($now.Count -eq 1) {
        $port = $now[0].Port
    } elseif ($target) {
        $port = ($now | Where-Object { $_.Port -eq $target.Port } | Select-Object -First 1).Port
    }
    if (-not $port) { Start-Sleep -Milliseconds 100 }
}
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
