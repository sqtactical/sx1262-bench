"""Speed test for the Jentastic/Nexus firmware's alt modems (SETFSK / SETFLRC), same tests as mesh_speed.py.

  python jt_speed.py --modes "LORA;FLRC 1300 2;FSK 1000 250" [--n 20] [--sizes 200,80]

Each mode is sent to both nodes as a local Nexus command ("/nexus-local SETFLRC 1300 2", "SETFSK OFF" for LORA);
the firmware applies it 5 s later and keeps it in RAM only. Results go to jt_results.json.
"""
import argparse, json, os, statistics, threading, time
from pubsub import pub
import mesh_speed
from mesh_speed import Counter, ack_test, connect, ensure_keys, stream_test

HERE = os.path.dirname(os.path.abspath(__file__))


class Replies:
    """collects text messages each interface hands to the PC (Nexus command replies)"""

    def __init__(self):
        self.lock = threading.Lock()
        self.texts = []
        pub.subscribe(self.on_text, "meshtastic.receive.text")

    def on_text(self, packet, interface):
        with self.lock:
            self.texts.append((interface, packet.get("decoded", {}).get("text", "")))

    def take(self, iface):
        with self.lock:
            mine = [t for i, t in self.texts if i is iface]
            self.texts = [(i, t) for i, t in self.texts if i is not iface]
        return mine


def nexus(iface, replies, cmd, wait=2.0):
    iface.sendText("/nexus-local " + cmd)
    time.sleep(wait)
    return replies.take(iface)


def set_mode(a, b, replies, mode):
    cmd = "SETFSK OFF" if mode == "LORA" else "SET" + mode
    out = {}
    for name, iface in (("A", a), ("B", b)):
        out[name] = nexus(iface, replies, cmd, wait=0.5)
    time.sleep(7)  # the firmware replies first and re-programs the radio 5 s later
    for name, iface in (("A", a), ("B", b)):
        out[name] += nexus(iface, replies, "SETFLRC" if "FLRC" in mode else "SETFSK")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="COM14")
    ap.add_argument("--b", default="COM27")
    ap.add_argument("--modes", default="LORA;FLRC 1300 2;FSK 1000 250")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--sizes", default="200,80")
    ap.add_argument("--out", default="jt_results.json")
    args = ap.parse_args()

    path = os.path.join(HERE, args.out)
    results = json.load(open(path, encoding="utf8")) if os.path.exists(path) else {}
    replies = Replies()
    a, b = connect(args.a), connect(args.b)
    try:
        bnum = b.myInfo.my_node_num
        counter = Counter(b)
        time.sleep(3)
        ensure_keys(a, b)
        for mode in args.modes.split(";"):
            status = set_mode(a, b, replies, mode)
            print(f"\n== {mode}: {json.dumps(status, ensure_ascii=False)}", flush=True)
            for size in (int(s) for s in args.sizes.split(",")):
                ack_test(a, bnum, 2, size, 10)  # warm-up
                lat = ack_test(a, bnum, args.n, size, 15)
                ok = [x for x in lat if x is not None]
                ack = {"n": args.n, "acked": len(ok), "lat_median_ms": statistics.median(ok) * 1000 if ok else None}
                stream = stream_test(a, counter, args.n, size, settle=6)
                rssi = [m[0] for m in counter.meta if m[0]]
                results[f"{mode} / {size}B"] = {"mode": mode, "size": size, "ack": ack, "stream": stream, "status": status,
                                                 "rssi_median": statistics.median(rssi) if rssi else None}
                print(f"{mode:>16} {size:3}B | ACK {ack['acked']:2}/{args.n}, median {ack['lat_median_ms'] or 0:6.0f} ms | "
                      f"stream {stream['received']:2}/{stream['sent']} in {stream['total_s']:4.1f} s, {stream['kbps']:5.1f} kb/s",
                      flush=True)
                json.dump(results, open(path, "w", encoding="utf8"), indent=1, ensure_ascii=False)
        set_mode(a, b, replies, "LORA")
    finally:
        counter.close()
        a.close()
        b.close()


if __name__ == "__main__":
    main()
