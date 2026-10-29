"""Throughput past the Python library's ~50 ms per message: A gets raw ToRadio frames written straight to its USB
serial at a fixed interval; B counts what arrives through the normal library.

  python raw_rate.py --a COM38 --b COM39 --intervals 40,20,10,5 [--n 60] [--size 220] [--mode "FSK 300"]
                     [--preset GFSK_TURBO]

--mode sends a jentastic Nexus command to both nodes first ("FSK 300", "LORA" = SETFSK OFF);
--preset switches both nodes to a modem preset (stock/develop firmware). Results go to raw_rate_results.json.
"""
import argparse, json, os, random, threading, time
import serial
from pubsub import pub
from meshtastic.protobuf import mesh_pb2
import mesh_speed
from mesh_speed import connect

HERE = os.path.dirname(os.path.abspath(__file__))
PRIVATE_APP = 256


def frame(payload):
    tr = mesh_pb2.ToRadio()
    p = tr.packet
    p.to = 0xFFFFFFFF
    p.id = random.getrandbits(31) | 1
    p.channel = 0
    p.hop_limit = 3
    p.want_ack = False
    p.decoded.portnum = PRIVATE_APP
    p.decoded.payload = payload
    data = tr.SerializeToString()
    return b"\x94\xc3" + len(data).to_bytes(2, "big") + data


class Rx:
    def __init__(self, iface, tag):
        self.iface, self.tag, self.times = iface, tag, []
        pub.subscribe(self.on_rx, "meshtastic.receive")

    def on_rx(self, packet, interface):
        d = packet.get("decoded", {})
        if interface is self.iface and d.get("portnum") == "PRIVATE_APP" and d.get("payload", b"")[:4] == self.tag:
            self.times.append(time.time())

    def close(self):
        pub.unsubscribe(self.on_rx, "meshtastic.receive")


def drain(s, stop):
    while not stop.is_set():
        try:
            s.read(4096)
        except Exception:
            return


def run(a_port, b, n, size, interval_ms):
    tag = os.urandom(4)
    rx = Rx(b, tag)
    frames = [frame(tag + i.to_bytes(2, "big") + os.urandom(size - 6)) for i in range(n)]
    s = serial.Serial(a_port, 115200, timeout=0.05)
    stop = threading.Event()
    th = threading.Thread(target=drain, args=(s, stop), daemon=True)
    th.start()
    t0 = time.time()
    for i, f in enumerate(frames):
        s.write(f)
        due = t0 + (i + 1) * interval_ms / 1000
        while time.time() < due:
            time.sleep(0.0005)
    t_sent = time.time()
    last, deadline = len(rx.times), time.time() + 5
    while time.time() < deadline:
        time.sleep(0.2)
        if len(rx.times) != last:
            last, deadline = len(rx.times), time.time() + 3
    stop.set()
    s.close()
    rx.close()
    got = len(rx.times)
    span = (rx.times[-1] - t0) if got else 0
    return {"interval_ms": interval_ms, "offered_kbps": size * 8 / interval_ms, "sent": n, "received": got,
            "send_s": round(t_sent - t0, 2), "span_s": round(span, 2),
            "kbps": round(got * size * 8 / span / 1000, 1) if got else 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="COM38")
    ap.add_argument("--b", default="COM39")
    ap.add_argument("--intervals", default="40,20,10,5")
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--size", type=int, default=220)
    ap.add_argument("--mode", help='jentastic: "FSK 300", "LORA", ...')
    ap.add_argument("--preset", help="modem preset name from mesh_speed.PRESETS")
    ap.add_argument("--label")
    ap.add_argument("--out", default="raw_rate_results.json")
    args = ap.parse_args()

    if args.preset:
        if any([mesh_speed.set_preset(p, mesh_speed.PRESETS[args.preset]) for p in (args.a, args.b)]):
            time.sleep(30)
    if args.mode:
        cmd = "SETFSK OFF" if args.mode == "LORA" else "SET" + args.mode
        for p in (args.a, args.b):
            i = connect(p)
            i.sendText("/nexus-local " + cmd)
            time.sleep(1)
            i.close()
        time.sleep(8)
    label = args.label or args.mode or args.preset or "current"
    path = os.path.join(HERE, args.out)
    results = json.load(open(path)) if os.path.exists(path) else {}
    b = connect(args.b)
    time.sleep(3)
    try:
        for iv in [float(x) for x in args.intervals.split(",")]:
            r = run(args.a, b, args.n, args.size, iv)
            results.setdefault(label, {})[str(iv)] = r
            print(f"{label:>14} every {iv:4.0f} ms (offered {r['offered_kbps']:5.1f} kb/s): {r['received']:3d}/{r['sent']} "
                  f"received, {r['kbps']:5.1f} kb/s over {r['span_s']:5.2f} s (sending took {r['send_s']:.2f} s)",
                  flush=True)
            json.dump(results, open(path, "w"), indent=1)
            time.sleep(3)
    finally:
        b.close()


if __name__ == "__main__":
    main()
