#!/usr/bin/env python3
import argparse, json, sys
try:
    import zmq  # type: ignore
except Exception:
    zmq = None

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--metadata", default="tcp://127.0.0.1:5600")
    ap.add_argument("--topic", default="camera.health")
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()
    if zmq is None:
        print("pyzmq is required: pip install pyzmq", file=sys.stderr)
        return 2
    ctx = zmq.Context.instance(); sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, args.topic); sock.connect(args.metadata)
    while True:
        _, payload = sock.recv_multipart()
        msg = json.loads(payload.decode())
        print(json.dumps(msg, indent=2, sort_keys=True))
        if args.once: return 0
if __name__ == "__main__": raise SystemExit(main())
