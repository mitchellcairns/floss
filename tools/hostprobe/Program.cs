// FLOSS - Free, Lightweight, Open Short-range Stack
// SPDX-License-Identifier: Unlicense
//
// floss_hostprobe - Windows host-side probe for measuring a FLOSS device:
//   scan|pair|unpair|paired <name> [--le]   find / pair (auto-confirm) / forget a device
//   conns                                  list paired devices and whether connected
//   pads <seconds>                         read the first game controller through the
//                                          OS game-input API and analyse report timing
//
// Report timing analysis relies on the example gamepad's demo mode, where one
// axis carries a 1 kHz millisecond counter. From it the probe recovers, per
// report: how many device milliseconds passed (coalescing / loss), host
// inter-arrival, and latency jitter = host time - device time after removing
// clock drift (0 = the fastest delivery observed).
//
// Build: dotnet build -c Release -o out   (needs the .NET 8+ SDK, Windows 10+).
// The OS radio API namespaces below are third-party identifiers, used only to
// talk to the platform.
using System.Diagnostics;
using Windows.Devices.Enumeration;
using Windows.Gaming.Input;
using BtDev = Windows.Devices.Bluetooth.BluetoothDevice;
using LeDev = Windows.Devices.Bluetooth.BluetoothLEDevice;

static class P {
    static async Task<int> Main(string[] a) {
        string cmd = a.Length > 0 ? a[0] : "help";
        string name = a.Length > 1 ? a[1] : "FLOSS";
        bool le = a.Contains("--le");
        switch (cmd) {
            case "scan": return await Scan(name, le, false);
            case "pair": return await Pair(name, le);
            case "unpair": return await Unpair(name, le);
            case "paired": return await Scan(name, le, true);
            case "pads": return await Pads(a.Length > 1 ? int.Parse(a[1]) : 5);
            case "conns": return await Conns();
            default: Console.WriteLine("scan|pair|unpair|paired <name> [--le] ; pads <seconds>"); return 1;
        }
    }
    static string Sel(bool le, bool paired) => le ? LeDev.GetDeviceSelectorFromPairingState(paired) : BtDev.GetDeviceSelectorFromPairingState(paired);
    static async Task<DeviceInformation> Find(string name, bool le, bool paired, int seconds = 30) {
        var sw = Stopwatch.StartNew();
        while (sw.Elapsed.TotalSeconds < seconds) {
            var all = await DeviceInformation.FindAllAsync(Sel(le, paired));
            var d = all.FirstOrDefault(x => x.Name.Contains(name));
            if (d != null) return d;
            await Task.Delay(1500);
        }
        return null;
    }
    static async Task<int> Scan(string name, bool le, bool paired) {
        var d = await Find(name, le, paired);
        Console.WriteLine(d == null ? "not found" : $"FOUND '{d.Name}' paired={d.Pairing.IsPaired} id={d.Id}");
        return d == null ? 1 : 0;
    }
    static async Task<int> Pair(string name, bool le) {
        var d = await Find(name, le, false);
        if (d == null) { Console.WriteLine("not found (already paired?)"); return 1; }
        Console.WriteLine($"pairing '{d.Name}' {d.Id}");
        var custom = d.Pairing.Custom;
        custom.PairingRequested += (s, e) => {
            Console.WriteLine($"  pairing requested: kind={e.PairingKind} pin={e.Pin}");
            e.Accept();
        };
        var kinds = DevicePairingKinds.ConfirmOnly | DevicePairingKinds.ConfirmPinMatch | DevicePairingKinds.DisplayPin | DevicePairingKinds.ProvidePin;
        var sw = Stopwatch.StartNew();
        var r = await custom.PairAsync(kinds, DevicePairingProtectionLevel.None);
        Console.WriteLine($"pair result: {r.Status} protection={r.ProtectionLevelUsed} in {sw.ElapsedMilliseconds} ms");
        return r.Status == DevicePairingResultStatus.Paired || r.Status == DevicePairingResultStatus.AlreadyPaired ? 0 : 1;
    }
    static async Task<int> Unpair(string name, bool le) {
        var d = await Find(name, le, true, 5);
        if (d == null) { Console.WriteLine("no paired device"); return 0; }
        var r = await d.Pairing.UnpairAsync();
        Console.WriteLine($"unpair '{d.Name}': {r.Status}");
        return 0;
    }
    static async Task<int> Conns() {
        foreach (var d in await DeviceInformation.FindAllAsync(BtDev.GetDeviceSelectorFromPairingState(true))) {
            try { var b = await BtDev.FromIdAsync(d.Id); Console.WriteLine($"classic '{d.Name}' {b?.ConnectionStatus}"); } catch (Exception e) { Console.WriteLine($"classic '{d.Name}' ? {e.Message}"); }
        }
        foreach (var d in await DeviceInformation.FindAllAsync(LeDev.GetDeviceSelectorFromPairingState(true))) {
            try { var b = await LeDev.FromIdAsync(d.Id); Console.WriteLine($"le      '{d.Name}' {b?.ConnectionStatus}"); } catch (Exception e) { Console.WriteLine($"le '{d.Name}' ? {e.Message}"); }
        }
        return 0;
    }
    static async Task<int> Pads(int seconds) {
        // RawGameController sees any HID game controller, including ours.
        var sw = Stopwatch.StartNew();
        RawGameController pad = null;
        while (sw.Elapsed.TotalSeconds < 5 && pad == null) {
            pad = RawGameController.RawGameControllers.FirstOrDefault();
            if (pad == null) await Task.Delay(200);
        }
        if (pad == null) { Console.WriteLine("no game controller"); return 1; }
        Console.WriteLine($"controller: '{pad.DisplayName}' vid=0x{pad.HardwareVendorId:X4} pid=0x{pad.HardwareProductId:X4} buttons={pad.ButtonCount} axes={pad.AxisCount}");
        var buttons = new bool[pad.ButtonCount]; var sws = new GameControllerSwitchPosition[pad.SwitchCount]; var axes = new double[pad.AxisCount];
        var hostT = new List<double>(); var vals = new List<double[]>();
        ulong lastTs = 0; double[] last = null;
        var t = Stopwatch.StartNew();
        double tickMs = 1000.0 / Stopwatch.Frequency;
        while (t.Elapsed.TotalSeconds < seconds) {
            ulong ts = pad.GetCurrentReading(buttons, sws, axes);
            if (ts != lastTs && (last == null || !axes.SequenceEqual(last))) {
                lastTs = ts; last = (double[])axes.Clone();
                hostT.Add(Stopwatch.GetTimestamp() * tickMs); vals.Add(last);
            }
        }
        int n = hostT.Count;
        Console.WriteLine($"distinct reports seen: {n} in {seconds}s = {n / (double)seconds:F1}/s");
        if (n < 10) return 1;
        // Find the millisecond-counter axis: the one whose decoded deltas are mostly small positives.
        int best = -1; double bestScore = -1;
        for (int a = 0; a < pad.AxisCount; a++) {
            int good = 0;
            for (int i = 1; i < n; i++) { long d = Dms(vals[i][a], vals[i - 1][a]); if (d >= 1 && d <= 50) good++; }
            if (good > bestScore) { bestScore = good; best = a; }
        }
        Console.WriteLine($"counter axis {best} ({bestScore / (n - 1):P0} monotonic)");
        // Unwrap device ms, host intervals, advances.
        var dev = new double[n]; dev[0] = 0;
        var hostIv = new List<double>(); var adv = new List<long>();
        for (int i = 1; i < n; i++) {
            long d = Dms(vals[i][best], vals[i - 1][best]);
            dev[i] = dev[i - 1] + d; adv.Add(d); hostIv.Add(hostT[i] - hostT[i - 1]);
        }
        double span = dev[n - 1];
        Console.WriteLine($"device ms covered {span:F0}; reports delivered {n} => {n / span * 1000:F0}/s of the 1000/s generated ({n / span:P1}), coalesced {(span - n) / span:P1}");
        Pct("host inter-arrival ms", hostIv);
        var advD = adv.Select(x => (double)x).ToList();
        Pct("device ms between delivered reports", advD);
        // Latency jitter: host time - device time, linear drift removed.
        double mx = dev.Average(), my = hostT.Average(), sxy = 0, sxx = 0;
        for (int i = 0; i < n; i++) { sxy += (dev[i] - mx) * (hostT[i] - my); sxx += (dev[i] - mx) * (dev[i] - mx); }
        double k = sxy / sxx, b = my - k * mx;
        var res = new List<double>();
        for (int i = 0; i < n; i++) res.Add(hostT[i] - (k * dev[i] + b));
        double rmin = res.Min();
        var resRel = res.Select(r => r - rmin).ToList();   // 0 = fastest observed delivery
        Pct("latency above best-case ms (jitter)", resRel);
        Console.WriteLine($"clock ratio host/device {k:F5}");
        // Gap structure: when do >3 ms holes happen, and are they periodic?
        var gaps = new List<(double at, double len)>();
        for (int i = 1; i < n; i++) if (hostT[i] - hostT[i - 1] > 3.0) gaps.Add((hostT[i - 1] - hostT[0], hostT[i] - hostT[i - 1]));
        Console.WriteLine($"gaps > 3 ms: {gaps.Count} ({gaps.Count / (double)seconds:F1}/s), total {gaps.Sum(g => g.len):F0} ms of {seconds * 1000} ms");
        if (gaps.Count > 2) {
            var spacing = new List<double>();
            for (int i = 1; i < gaps.Count; i++) spacing.Add(gaps[i].at - gaps[i - 1].at);
            Pct("spacing between gaps ms", spacing);
            Pct("gap length ms", gaps.Select(g => g.len).ToList());
            Console.WriteLine("first gaps (t ms: len): " + string.Join("  ", gaps.Take(14).Select(g => $"{g.at:F0}:{g.len:F1}")));
        }
        return 0;
    }
    static long Dms(double cur, double prev) {
        long c = (long)Math.Round(cur * 65534.0), p = (long)Math.Round(prev * 65534.0);
        long d = c - p; if (d < -30000) d += 65535; return d;
    }
    static void Pct(string name, List<double> v) {
        var s = v.OrderBy(x => x).ToList(); int n = s.Count;
        double P(double q) => s[Math.Min(n - 1, (int)(q * n))];
        double mean = s.Average(); double sd = Math.Sqrt(s.Select(x => (x - mean) * (x - mean)).Average());
        Console.WriteLine($"  {name,-38} min {s[0],6:F2} p50 {P(0.5),6:F2} p90 {P(0.9),6:F2} p99 {P(0.99),6:F2} max {s[n - 1],6:F2} mean {mean,6:F2} sd {sd,5:F2}");
    }
}
