"""Send a file between two SX1262 modem boards and measure throughput in every mode of a modulation.

  python sx1262_host.py --tx COM41 --rx COM40 [--modem lora|gfsk] [--file path] [--size 204800] [--secs 8]
                        [--modes all|fastest|slowest|<name,...>] [--pwr -9] [--freq 869525] [--win 64] [--relay] [-v]

For each mode (slowest -> fastest nominal bit rate):
  1. BURST: raw back-to-back full-size packets, no ACKs  -> raw air throughput + packet loss
  2. FILE : USB -> board A -> radio (block ARQ) -> board B -> USB, checked with SHA-256
With --relay the FILE step is replaced by the mesh-style relay (per packet: data, hop ACK, forward, ACK).
The file is the first --size bytes of --file (or random data), shortened so each test takes about --secs.
Firmware: firmware/sx1262_modem (same protocol as the SX1280 bench modem).
"""
import argparse, hashlib, json, os, threading, time
import serial

HERE = os.path.dirname(os.path.abspath(__file__))
# SX126x GFSK receiver bandwidths (kHz DSB) and their ModParam5 codes (datasheet table 13-45)
RXBW = [(4.8, 0x1F), (5.8, 0x17), (7.3, 0x0F), (9.7, 0x1E), (11.7, 0x16), (14.6, 0x0E), (19.5, 0x1D), (23.4, 0x15),
        (29.3, 0x0D), (39.0, 0x1C), (46.9, 0x14), (58.6, 0x0C), (78.2, 0x1B), (93.8, 0x13), (117.3, 0x0B),
        (156.2, 0x1A), (187.2, 0x12), (234.3, 0x0A), (312.0, 0x19), (373.6, 0x11), (467.0, 0x09)]
GFSK_KBPS = [4.8, 9.6, 19.2, 38.4, 50, 100, 150, 200, 250, 300]
LORA_BW = {0x04: 125.0, 0x05: 250.0, 0x06: 500.0}                                           # code: kHz


def gfsk_mode(kbps):
    """deviation: modulation index 1 up to 50 kb/s, 0.5 above; RX filter: smallest step over 1.1 x Carson"""
    dev = kbps / 2 if kbps <= 50 else kbps / 4
    need = (kbps + 2 * dev) * 1.1
    bw, code = next(((b, c) for b, c in RXBW if b >= need), RXBW[-1])
    return dev, bw, code


def modes(modem, pl):
    """[(name, CFG args, nominal kb/s)] sorted by nominal bit rate"""
    m = []
    if modem == "gfsk":
        for kb in GFSK_KBPS:
            dev, bw, code = gfsk_mode(kb)
            m.append((f"{kb:g}k dev{dev:g} BW{bw:g}", (0, int(kb * 1000), int(dev * 1000), code, 32, pl), kb))
    else:
        for bw, khz in LORA_BW.items():
            for sf in range(5, 13):
                kb = sf * khz / 2 ** sf * 4 / 5
                m.append((f"SF{sf} BW{khz:g}", (1, sf, bw, 1, 12, pl), kb))    # CR 4/5, 12-symbol preamble
    return sorted(m, key=lambda x: (x[2], x[0]))


def cmd(s, line, expect, timeout=3):
    s.reset_input_buffer()
    s.write(line.encode() + b"\n")
    return cmd_wait(s, expect, timeout)


def cmd_wait(s, expect, timeout):
    t = time.time()
    while time.time() - t < timeout:
        r = s.readline().decode(errors="replace").strip()
        if r.startswith(expect):
            return r
        if r.startswith(("FAIL", "ERR")):
            raise RuntimeError(f"{s.port}: {r}")
    raise TimeoutError(f"{s.port}: no '{expect}'")


def kv(line):
    return {k: v for k, v in (p.split("=", 1) for p in line.split() if "=" in p)}


def stop_rx(rx):
    rx.write(b"\n")
    return cmd_wait(rx, "RXSTAT", 10)


def burst(tx, rx, toa_us, pl, secs):
    n = int(min(3000, max(3, secs * 1e6 / (toa_us + 500))))
    rx.reset_input_buffer()
    rx.write(b"RX\n"); time.sleep(0.15)
    r = cmd(tx, f"BURST {n}", "BURSTDONE", timeout=10 + n * toa_us * 1.5e-6)
    prof = cmd_wait(tx, "PROF", 2)
    time.sleep(0.1 + 2 * toa_us * 1e-6)
    st = kv(stop_rx(rx))
    us = int(kv(r)["us"])
    got = int(st["raw"])
    return {"n": n, "rx": got, "loss_pct": 100 * (n - got) / n, "crc": int(st["crc"]), "rssi": int(st["rssi"]),
            "rawbad": int(st["rawbad"]), "pkt_us": us / n, "toa_us": toa_us, "raw_kbps": n * pl * 8 / us * 1000,
            "prof": prof}


def transfer(tx, rx, data, out_path, nominal_kbps):
    size = len(data)
    res = {}
    rx.reset_input_buffer()
    rx.write(b"RX\n"); time.sleep(0.15)
    tx.reset_input_buffer()
    limit = 30 + size * 8 / (nominal_kbps * 1000) * 4

    def reader():
        try:
            hdr = cmd_wait(rx, "START", limit)
            buf = bytearray()
            while len(buf) < size:
                chunk = rx.read(min(65536, size - len(buf)))
                if not chunk and time.time() - t_start > limit:
                    raise TimeoutError(f"rx stalled at {len(buf)}/{size}")
                buf += chunk
            res["t_rx_end"] = time.perf_counter()
            res["rx_end"] = cmd_wait(rx, "END", 10)
            res["rx_start"] = hdr
            res["data"] = bytes(buf)
        except Exception as e:
            res["rx_err"] = str(e)

    t_start = time.time()
    th = threading.Thread(target=reader)
    th.start()
    t0 = time.perf_counter()
    tx.write(f"TX {size}\n".encode())
    tx.write(data)
    done = None
    while time.time() - t_start < limit:
        r = tx.readline().decode(errors="replace").strip()
        if r.startswith(("DONE", "FAIL")):
            done = r
            break
    prof = cmd_wait(tx, "PROF", 2) if done and done.startswith("DONE") else ""
    th.join(timeout=15)
    st = stop_rx(rx)
    if not done or not done.startswith("DONE") or "data" not in res:
        raise RuntimeError(f"transfer failed: tx={done} rx={res.get('rx_err')} rxstat={st}")
    ok = hashlib.sha256(res["data"]).digest() == hashlib.sha256(data).digest()
    with open(out_path, "wb") as f:
        f.write(res["data"])
    d, e = kv(done), kv(res["rx_end"])
    host_s = res["t_rx_end"] - t0
    return {"size": size, "sha_ok": ok, "tx_kbps": float(d["kbps"]), "rx_kbps": float(e["kbps"]),
            "host_kbps": size * 8 / host_s / 1000, "host_s": host_s,
            "pkts": int(d["pkts"]), "retx": int(d["retx"]), "polls": int(d["polls"]), "tmos": int(d["tmos"]),
            "crcerr": int(e["crcerr"]), "rssi": int(e["rssi"]),
            "t_start": int(d["t_start"]), "t_data": int(d["t_data"]), "t_ack": int(d["t_ack"]),
            "t_usbwait": int(d["t_usbwait"]), "radio_us": int(d["us"]), "blocks": int(d["blocks"]), "prof": prof}


def relay(tx, rx, data, nominal_kbps):
    """Mesh-style: per packet A->B data, B->A hop ACK, B forwards (->A as next node), A->B ACK.
    A compares every forwarded copy with what it sent (end-to-end check, reported as mismatch=)."""
    size = len(data)
    rx.reset_input_buffer()
    rx.write(b"RX\n"); time.sleep(0.15)
    tx.reset_input_buffer()
    limit = 30 + size * 8 / (nominal_kbps * 1000) * 10
    th = threading.Thread(target=lambda: (tx.write(f"RELAY {size}\n".encode()), tx.write(data)), daemon=True)
    t0 = time.perf_counter()
    th.start()
    done = None
    while time.perf_counter() - t0 < limit:
        r = tx.readline().decode(errors="replace").strip()
        if r.startswith(("DONE", "FAIL")):
            done = r
            break
    t_end = time.perf_counter()
    th.join(timeout=10)
    st = kv(stop_rx(rx))
    if not done or not done.startswith("DONE"):
        raise RuntimeError(f"relay failed: tx={done} rxstat={st}")
    d = kv(done)
    host_s = t_end - t0
    return {"size": size, "sha_ok": int(d["mismatch"]) == 0, "mismatch": int(d["mismatch"]),
            "e2e_kbps": size * 8 / host_s / 1000, "radio_kbps": float(d["kbps"]), "host_s": host_s,
            "pkts": int(d["pkts"]), "retries": int(d["retries"]), "dupfwd": int(d["dupfwd"]),
            "radio_resets": int(d["radioResets"]), "relayed": int(st["relayed"]), "fwdretx": int(st["fwdretx"])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tx", default="COM41")
    ap.add_argument("--rx", default="COM40")
    ap.add_argument("--modem", default="gfsk", choices=["lora", "gfsk"])
    ap.add_argument("--file")
    ap.add_argument("--size", type=int, default=204800, help="max file bytes")
    ap.add_argument("--secs", type=float, default=8, help="target seconds per file test (shrinks the file)")
    ap.add_argument("--pl", type=int, default=255, help="max payload bytes (16-255)")
    ap.add_argument("--pwr", type=int, default=-9, help="SX1262 output power in dBm (-9..22); keep it low on a bench")
    ap.add_argument("--freq", type=int, default=869525, help="kHz")
    ap.add_argument("--modes", default="all")
    ap.add_argument("--pipe", type=int, default=1, help="preload the next packet while one is on air (pl <= 127)")
    ap.add_argument("--win", type=int, default=64, help="packets per ACK: 1 = every packet acknowledged (mesh-style)")
    ap.add_argument("--range", default=None, help="slice of the sorted mode list, e.g. 0:8")
    ap.add_argument("--relay", action="store_true", help="mesh-style test: TX, ACK, the other node forwards, ACK")
    ap.add_argument("-v", action="store_true", help="print firmware timing breakdown")
    ap.add_argument("--out")
    a = ap.parse_args()
    out = a.out or (f"results_{a.modem}_relay.json" if a.relay else
                    f"results_{a.modem}.json" if a.win == 64 else f"results_{a.modem}_win{a.win}.json")

    full = open(a.file, "rb").read()[:a.size] if a.file else os.urandom(a.size)
    tx = serial.Serial(a.tx, 115200, timeout=2, write_timeout=None)
    rx = serial.Serial(a.rx, 115200, timeout=2)
    time.sleep(0.3)
    for s in (tx, rx):
        s.write(b"\n"); time.sleep(0.05); s.reset_input_buffer()
        print(s.port, cmd(s, "PING", "PONG"))
        print(s.port, cmd(s, f"OPT {a.pipe} {a.win}", "OK"))

    ms = modes(a.modem, a.pl)
    if a.modes == "fastest": ms = ms[-1:]
    elif a.modes == "slowest": ms = ms[:1]
    elif a.modes != "all": ms = [m for m in ms if m[0] in a.modes.split(",")]   # e.g. "SF7 BW250,SF9 BW125"
    if a.range:
        i, j = (int(x) if x else None for x in a.range.split(":"))
        ms = ms[i:j]
    os.makedirs(os.path.join(HERE, "received"), exist_ok=True)
    results = []
    print(f"modem {a.modem}, payload {a.pl} B, ACK every {a.win} packets, {a.pwr} dBm at the SX1262, {a.freq} kHz")
    for name, (modem, p1, p2, p3, pre, mpl), nominal in ms:
        for s in (tx, rx):
            ok = cmd(s, f"CFG {modem} {p1} {p2} {p3:#x} {a.pwr} {pre} {mpl} {a.freq}", "OK")
        toa = int(kv(ok)["toa"].rstrip("us"))
        size = min(len(full), max(2 * (mpl - 4), int(nominal * 1000 / 8 * a.secs)))
        data = full[:size]
        row = {"mode": name, "nominal_kbps": nominal, "pl": mpl, "toa_us": toa}
        if a.relay:
            data = data[:max(2 * (mpl - 4), len(data) // 3)]   # each byte crosses the air twice
            try:
                r = relay(tx, rx, data, nominal)
            except Exception as e:
                r = {"error": str(e)}
            row["relay"] = r
            results.append(row)
            print(f"{name:>22} | nominal {nominal:7.2f} kb/s | relay {r.get('size', 0):6d} B end-to-end "
                  f"{r.get('e2e_kbps', 0):7.2f} kb/s ({100 * r.get('e2e_kbps', 0) / nominal:4.1f}%) "
                  f"radio {r.get('radio_kbps', 0):7.2f} sha {'OK' if r.get('sha_ok') else 'FAIL'} "
                  f"retries {r.get('retries', '-')} fwdretx {r.get('fwdretx', '-')} {r.get('error', '')}", flush=True)
        else:
            try:
                row["burst"] = burst(tx, rx, toa, mpl, min(a.secs, 3))
            except Exception as e:
                row["burst"] = {"error": str(e)}
            try:
                row["file"] = transfer(tx, rx, data, os.path.join(HERE, "received", f"rx_{a.modem}_{p1}_{p2}_{p3:02x}.bin"),
                                       nominal)
            except Exception as e:
                row["file"] = {"error": str(e)}
            results.append(row)
            b, f = row["burst"], row["file"]
            pct = 100 * f.get("host_kbps", 0) / nominal
            print(f"{name:>22} | nominal {nominal:7.2f} kb/s | burst {b.get('raw_kbps', 0):7.2f} kb/s "
                  f"loss {b.get('loss_pct', -1):5.1f}% rssi {b.get('rssi', 0)} | file {f.get('size', 0):6d} B "
                  f"{f.get('host_kbps', 0):7.2f} kb/s ({pct:4.1f}%) radio {f.get('tx_kbps', 0):7.2f} "
                  f"sha {'OK' if f.get('sha_ok') else 'FAIL'} retx {f.get('retx', '-')} tmo {f.get('tmos', '-')} "
                  f"{f.get('error', '') or b.get('error', '')}", flush=True)
            if a.v:
                print(f"{'':>25}burst {b.get('prof', '')} rawbad={b.get('rawbad')}")
                if "t_data" in f:
                    print(f"{'':>25}file  {f['prof']}")
                    print(f"{'':>25}file  radio={f['radio_us']}us start={f['t_start']} data={f['t_data']} "
                          f"ack={f['t_ack']} usbwait={f['t_usbwait']} blocks={f['blocks']} crcerr={f['crcerr']}")
        with open(os.path.join(HERE, out), "w") as fh:
            json.dump({"modem": a.modem, "pl": a.pl, "pwr": a.pwr, "freq_khz": a.freq, "win": a.win,
                       "relay": a.relay, "results": results}, fh, indent=1)


if __name__ == "__main__":
    main()
