# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# experiment.ps1 - Latency experiments against the Pico W gamepad firmware.
# For each config string: send its keys to the device console (live tuning
# keys, see ports/pico_w/README.md), settle, reset device stats, measure the
# host-side report timing with floss_hostprobe, then print device stats.
#
#   powershell -File tools/hostprobe/experiment.ps1 -Flash -Configs m,4,P -Seconds 10
param([string[]]$Configs = @("m"), [int]$Seconds = 8, [switch]$Flash)

$tool = Join-Path $PSScriptRoot "out\floss_hostprobe.exe"
$repo = Resolve-Path (Join-Path $PSScriptRoot "..\..")
if (-not (Test-Path $tool)) { Write-Error "build the probe first: dotnet build -c Release -o out (in tools/hostprobe)"; exit 1 }
if ($Flash) {
    & (Join-Path $repo "tools\pico_flash.ps1") -Uf2 (Join-Path $repo "build-pico\floss_gamepad.uf2") -ReadMs 0
    Start-Sleep -Seconds 7   # boot + automatic reconnect to the bonded host
}
$port = (Get-CimInstance Win32_SerialPort | Where-Object PNPDeviceID -match 'VID_2E8A').DeviceID
$p = New-Object System.IO.Ports.SerialPort $port, 115200
$p.DtrEnable = $true
$p.Open()
Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()
foreach ($cfg in $Configs) {
    Write-Output "################ config keys '$cfg'"
    foreach ($ch in $cfg.ToCharArray()) { $p.Write([string]$ch); Start-Sleep -Milliseconds 150 }
    Start-Sleep -Milliseconds 1500
    $o = $p.ReadExisting()
    ($o -split "`n" | Where-Object { $_ -match "tune:|QoS| W | E " }) | Select-Object -Last 3 | ForEach-Object { "device: $_" }
    $p.Write("r"); Start-Sleep -Milliseconds 200; $null = $p.ReadExisting()
    & $tool pads $Seconds 2>&1 | Where-Object { $_ -notmatch "^controller" }
    $p.Write("s"); Start-Sleep -Milliseconds 1200
    $o = $p.ReadExisting()
    ($o -split "`n" | Where-Object { $_ -match "hid_reports_sent|hid_send_deferred|coalesced|acl_ack_latency|transport_send_time|transport_poll_time|hid_report_age|hid_send_interval|poll_gap|timer_lateness|current_mode|->" }) | ForEach-Object { "device: $($_.TrimEnd())" }
}
$p.Close()
