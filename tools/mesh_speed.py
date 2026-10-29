"""Meshtastic speed test between two nodes for LoRa / FLRC / GFSK presets.

  python mesh_speed.py --a COM14 --b COM27 --presets SHORT_TURBO,FLRC_FAST,GFSK_TURBO [--n 30] [--size 200]

Per preset (both nodes are switched and rebooted):
  ack    : A sends N direct messages to B with want_ack, one at a time (like the app). Latency per message,
           delivered messages/s and payload kb/s.
  stream : A queues N broadcast messages without ACK as fast as the node's TX queue accepts them; B counts
           what arrives. Payload kb/s from first send to last reception.
Results go to mesh_results.json.
"""
import argparse, json, os, statistics, threading, time
import meshtastic.serial_interface
import meshtastic.stream_interface
from pubsub import pub


def _write_no_sleep(self, b):
    # the library sleeps 1 s after every write on Windows 11 (0.1 s elsewhere), which caps any test at 1 msg/s
    if self.stream:
        self.stream.write(b)
        self.stream.flush()


meshtastic.stream_interface.StreamInterface._writeBytes = _write_no_sleep

HERE = os.path.dirname(os.path.abspath(__file__))
PRESETS = {"LONG_FAST": 0, "MEDIUM_SLOW": 3, "MEDIUM_FAST": 4, "SHORT_SLOW": 5, "SHORT_FAST": 6, "SHORT_TURBO": 8,
           "LONG_TURBO": 9, "FLRC_TURBO": 64, "FLRC_FAST": 65, "FLRC_MEDIUM": 66, "FLRC_LONG": 67,
           "GFSK_TURBO": 72, "GFSK_FAST": 73, "GFSK_MEDIUM": 74, "GFSK_LONG": 75}
PRIVATE_APP = 256


def connect(port, tries=20):
    for _ in range(tries):
        try:
            return meshtastic.serial_interface.SerialInterface(port)
        except Exception:
            time.sleep(2)
    raise RuntimeError(f"cannot connect to {port}")


def set_preset(port, preset):
    """returns True if the node had to reboot"""
    iface = connect(port)
    try:
        lc = iface.localNode.localConfig.lora
        if lc.use_preset and lc.modem_preset == preset:
            return False
        lc.use_preset = True
        lc.modem_preset = preset
        iface.localNode.writeConfig("lora")
        time.sleep(1)
        return True
    finally:
        iface.close()


def ensure_keys(a, b):
    """DMs need each node to hold the other's public key: exchange NodeInfo if missing"""
    from google.protobuf.json_format import ParseDict
    from meshtastic.protobuf import mesh_pb2, portnums_pb2
    anum, bnum = a.myInfo.my_node_num, b.myInfo.my_node_num
    known = lambda x, num: bool(x.nodesByNum.get(num, {}).get("user", {}).get("publicKey"))
    if known(a, bnum) and known(b, anum):
        return
    for src, dnum in ((b, anum), (a, bnum)):
        me = src.nodesByNum[src.myInfo.my_node_num]["user"]
        u = ParseDict({k: v for k, v in me.items() if k != "raw"}, mesh_pb2.User(), ignore_unknown_fields=True)
        src.sendData(u.SerializeToString(), destinationId=dnum, portNum=portnums_pb2.PortNum.NODEINFO_APP,
                     wantResponse=True)
        time.sleep(4)


class Counter:
    def __init__(self, iface):
        self.iface = iface
        self.rx = []
        self.meta = []
        pub.subscribe(self.on_rx, "meshtastic.receive")

    def on_rx(self, packet, interface):
        if interface is not self.iface:
            return
        d = packet.get("decoded", {})
        if d.get("portnum") == "PRIVATE_APP":
            self.rx.append((time.time(), len(d.get("payload", b"")), d.get("payload", b"")[:4]))
            self.meta.append((packet.get("rxRssi"), packet.get("rxSnr")))

    def close(self):
        pub.unsubscribe(self.on_rx, "meshtastic.receive")


def ack_test(a, bnum, n, size, timeout):
    lat = []
    for i in range(n):
        ev = threading.Event()
        res = {}

        def cb(p, ev=ev, res=res):
            res["err"] = p.get("decoded", {}).get("routing", {}).get("errorReason", "NONE")
            ev.set()

        payload = i.to_bytes(4, "big") + os.urandom(size - 4)
        t0 = time.time()
        a.sendData(payload, destinationId=bnum, portNum=PRIVATE_APP, wantAck=True, onResponse=cb,
                   onResponseAckPermitted=True)
        ok = ev.wait(timeout) and res.get("err") == "NONE"
        lat.append(time.time() - t0 if ok else None)
    return lat


def stream_test(a, counter, n, size, settle):
    counter.rx.clear()
    t0 = time.time()
    for i in range(n):
        a.sendData(i.to_bytes(4, "big") + os.urandom(size - 4), portNum=PRIVATE_APP, wantAck=False)
    t_sent = time.time()
    last = len(counter.rx)
    deadline = time.time() + settle
    while time.time() < deadline:
        time.sleep(0.5)
        if len(counter.rx) != last:
            last = len(counter.rx)
            deadline = time.time() + settle
        if last >= n:
            break
    got = len(counter.rx)
    t_last = counter.rx[-1][0] if got else time.time()
    return {"sent": n, "received": got, "send_s": t_sent - t0, "total_s": t_last - t0,
            "kbps": got * size * 8 / (t_last - t0) / 1000 if got else 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="COM14")
    ap.add_argument("--b", default="COM27")
    ap.add_argument("--presets", default="SHORT_TURBO,FLRC_FAST,GFSK_TURBO")
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--size", type=int, default=200, help="payload bytes per message (max 233)")
    ap.add_argument("--timeout", type=float, default=30)
    ap.add_argument("--out", default="mesh_results.json")
    args = ap.parse_args()

    path = os.path.join(HERE, args.out)
    results = json.load(open(path)) if os.path.exists(path) else {}
    for name in args.presets.split(","):
        preset = PRESETS[name]
        rebooted = [set_preset(p, preset) for p in (args.a, args.b)]
        if any(rebooted):
            time.sleep(45)  # GPS probing after boot blocks the main loop for ~35 s
        a, b = connect(args.a), connect(args.b)
        try:
            bnum = b.myInfo.my_node_num
            counter = Counter(b)
            time.sleep(3)
            ensure_keys(a, b)
            ack_test(a, bnum, 3, args.size, args.timeout)  # warm-up: nodes are still busy right after a reboot
            lat = ack_test(a, bnum, args.n, args.size, args.timeout)
            ok = [x for x in lat if x is not None]
            ack = {"n": args.n, "acked": len(ok),
                   "lat_median_ms": statistics.median(ok) * 1000 if ok else None,
                   "lat_min_ms": min(ok) * 1000 if ok else None,
                   "msgs_per_s": len(ok) / sum(ok) if ok else 0,
                   "kbps": len(ok) * args.size * 8 / sum(ok) / 1000 if ok else 0}
            rssi = [m[0] for m in counter.meta if m[0]]
            stream = stream_test(a, counter, args.n, args.size, settle=max(5, args.timeout / 2))
            counter.close()
            results[name] = {"size": args.size, "ack": ack, "stream": stream,
                             "rssi_median": statistics.median(rssi) if rssi else None}
            print(f"{name:>12} | ACK: {ack['acked']}/{args.n} ok, median {ack['lat_median_ms'] or 0:7.0f} ms, "
                  f"{ack['msgs_per_s']:5.2f} msg/s, {ack['kbps']:6.2f} kb/s | stream: {stream['received']}/{stream['sent']} "
                  f"in {stream['total_s']:5.1f} s, {stream['kbps']:6.2f} kb/s | RSSI {results[name]['rssi_median']}",
                  flush=True)
        finally:
            a.close()
            b.close()
        json.dump(results, open(path, "w"), indent=1)


if __name__ == "__main__":
    main()
