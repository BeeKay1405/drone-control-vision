"""
shared_server.py -- runs on the PYNQ board. ONE server, ONE shared fabric,
serving BOTH the drone control loop and host-free MNIST vision.

It is a strict SUPERSET of your original drone_server.py: the /tick endpoint
behaves identically (same JSON in/out, including "error"), so your existing
drone_demo.py + sage_hardware.py work against it UNCHANGED. It just adds the
vision endpoints on top.

Only one process can own the PYNQ overlay, so the drone path and the vision
path must live in the same server -- that is the whole point of the shared
fabric.

Endpoints
---------
  GET  /health
       -> {"ok", "mode":"auto"|"manual", "tick_hz", "vision": true}
  POST /tick                          (IDENTICAL to drone_server.py)
       body: {"q":[w,x,y,z], "omega":[x,y,z], "thrust":int}
       -> {"motors":[..4], "torque":[x,y,z], "error":[x,y,z], "elapsed_us":int}
  POST /fault                         (DEBUG bitstream only)
       body: {"enable":0|1, "pe":0..15, "mode":0|1|2, "sage_en":0|1}
       -> {"fault_en", "fault_pe", "fault_mode", "sage_en"}  (read-back)
  POST /infer_raw                     (vision; what the webcam client sends)
       body: 676 int16 LE bytes
       -> {"prediction":int, "elapsed_ms":float}
  POST /infer
       body: {"pixels":[676 ints]}
       -> {"prediction":int, "elapsed_ms":float}
  GET  /test_image/<n>                (classify images.mem[n], if --images given)
       -> {"prediction":int, "ground_truth":int|null, "elapsed_ms":float}

Run on the board (NOT sudo):
  python3 shared_server.py --bitfile sage16_shared.bit
  python3 shared_server.py --bitfile sage16_shared.bit --images images.mem
  python3 shared_server.py --mock          # laptop dev, CPU math, no board
"""

import argparse
import json
import struct
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np


# ---- register map (matches control_top_shared.v == your control_top v4) ----
REG_Q_W      = 0x000
REG_Q_X      = 0x004
REG_Q_Y      = 0x008
REG_Q_Z      = 0x00C
REG_OMEGA_X  = 0x010
REG_OMEGA_Y  = 0x014
REG_OMEGA_Z  = 0x018
REG_THRUST   = 0x01C
REG_MOTOR0   = 0x100
REG_MOTOR1   = 0x104
REG_MOTOR2   = 0x108
REG_MOTOR3   = 0x10C
REG_TQX      = 0x110
REG_TQY      = 0x114
REG_TQZ      = 0x118
REG_ERR_X    = 0x11C
REG_ERR_Y    = 0x120
REG_ERR_Z    = 0x124
REG_CTRL     = 0x200
REG_STATUS   = 0x204
REG_AUTO_EN  = 0x208
REG_TICK_DIV = 0x20C
# debug fault-injection + self-heal (DEBUG bitstream only)
REG_DBG_FAULT_EN   = 0x210   # bit0 = inject
REG_DBG_FAULT_PE   = 0x214   # bits[3:0] = which PE (0..15)
REG_DBG_FAULT_MODE = 0x218   # bits[1:0] = 0 SA0, 1 SA1, 2 bit-28 flip
REG_SAGE_EN        = 0x21C   # bit0 = 1 -> ABFT repair the flagged PE
# vision (new)
REG_VIS_PIX    = 0x300   # bits[25:16]=index, bits[15:0]=int16 pixel
REG_VIS_CTRL   = 0x304   # bit0 = classify pulse
REG_VIS_STATUS = 0x308   # bit0=busy, bit1=valid, bits[5:2]=digit


def to_u16(x):  return int(x) & 0xFFFF
def from_s32(x):
    x &= 0xFFFFFFFF
    return x - 0x100000000 if x & 0x80000000 else x


# =============================================================================
# Hardware
# =============================================================================
class SharedIP:
    """Wraps the PYNQ MMIO/IP handle for sage16_shared_axi."""
    def __init__(self, ip):
        self._ip = ip
        self.manual_mode = False

    def write(self, a, v): self._ip.write(a, int(v) & 0xFFFFFFFF)
    def read(self, a):     return self._ip.read(a)

    # ---- control loop (same as drone_server.py) ----
    def arm_autotick(self, clk_hz, tick_hz):
        div = max(1, round(clk_hz / tick_hz) - 1)
        self.write(REG_TICK_DIV, div)
        self.write(REG_AUTO_EN, 1)
        actual = clk_hz / (div + 1)
        print(f"On-chip tick ARMED: {actual:.1f} Hz control loop")
        return actual

    def disarm_autotick(self):
        self.write(REG_AUTO_EN, 0)

    # ---- debug fault injection + self-heal (DEBUG bitstream only) ----
    def set_fault(self, enable, pe=0, mode=0, sage_en=0):
        """Assert/clear the fault-injection + repair debug registers.
        Order matters: set PE/mode/repair first, then flip enable last."""
        self.write(REG_DBG_FAULT_PE,   int(pe)   & 0xF)
        self.write(REG_DBG_FAULT_MODE, int(mode) & 0x3)
        self.write(REG_SAGE_EN,        int(sage_en) & 0x1)
        self.write(REG_DBG_FAULT_EN,   int(enable)  & 0x1)
        return {
            "fault_en":   self.read(REG_DBG_FAULT_EN)   & 0x1,
            "fault_pe":   self.read(REG_DBG_FAULT_PE)   & 0xF,
            "fault_mode": self.read(REG_DBG_FAULT_MODE) & 0x3,
            "sage_en":    self.read(REG_SAGE_EN)        & 0x1,
        }

    def _write_inputs(self, q, omega, thrust):
        self.write(REG_Q_W, to_u16(q[0]));  self.write(REG_Q_X, to_u16(q[1]))
        self.write(REG_Q_Y, to_u16(q[2]));  self.write(REG_Q_Z, to_u16(q[3]))
        self.write(REG_OMEGA_X, to_u16(omega[0]))
        self.write(REG_OMEGA_Y, to_u16(omega[1]))
        self.write(REG_OMEGA_Z, to_u16(omega[2]))
        self.write(REG_THRUST, to_u16(thrust))

    def _read_outputs(self):
        return {
            "motors": [from_s32(self.read(REG_MOTOR0)), from_s32(self.read(REG_MOTOR1)),
                       from_s32(self.read(REG_MOTOR2)), from_s32(self.read(REG_MOTOR3))],
            "torque": [from_s32(self.read(REG_TQX)), from_s32(self.read(REG_TQY)),
                       from_s32(self.read(REG_TQZ))],
            "error":  [from_s32(self.read(REG_ERR_X)), from_s32(self.read(REG_ERR_Y)),
                       from_s32(self.read(REG_ERR_Z))],
        }

    def tick(self, q, omega, thrust, timeout_us=10000):
        t0 = time.time()
        self._write_inputs(q, omega, thrust)
        if self.manual_mode:
            self.write(REG_CTRL, 1)
            deadline = t0 + timeout_us * 1e-6
            while True:
                if not (self.read(REG_STATUS) & 0x4):
                    break
                if time.time() > deadline:
                    raise TimeoutError("drone tick timeout")
        out = self._read_outputs()
        out["elapsed_us"] = (time.time() - t0) * 1e6
        return out

    # ---- vision (host-free CNN on the PL sequencer) ----
    def classify(self, pixels_26x26, timeout_ms=500):
        pix = np.asarray(pixels_26x26, dtype=np.int16).reshape(-1)
        assert pix.size == 676
        t0 = time.time()
        for idx, val in enumerate(pix):
            self.write(REG_VIS_PIX, ((idx & 0x3FF) << 16) | (int(val) & 0xFFFF))
        self.write(REG_VIS_CTRL, 1)
        deadline = time.time() + timeout_ms / 1000
        while time.time() < deadline:
            st = self.read(REG_VIS_STATUS)
            if not (st & 1) and (st & 2):
                return int((st >> 2) & 0xF), (time.time() - t0) * 1000
            time.sleep(0.0001)
        raise TimeoutError("vision classify timeout")


class MockIP:
    """CPU fallback (no board) -- drone returns zeros, vision uses numpy math."""
    def __init__(self, weights_dir="."):
        self.manual_mode = False
        from mnist_pipeline import MiniCNNWeights, CpuMockSage16, predict
        self._w = MiniCNNWeights(weights_dir)
        self._sage = CpuMockSage16()
        self._predict = predict
    def arm_autotick(self, *_): return 0.0
    def disarm_autotick(self): pass
    def set_fault(self, enable, pe=0, mode=0, sage_en=0):
        # no hardware fault path in mock; report the request back
        return {"fault_en": int(enable) & 1, "fault_pe": int(pe) & 0xF,
                "fault_mode": int(mode) & 3, "sage_en": int(sage_en) & 1}
    def tick(self, q, omega, thrust):
        return {"motors":[0,0,0,0], "torque":[0,0,0], "error":[0,0,0], "elapsed_us":0.0}
    def classify(self, pixels_26x26, **_):
        img = np.asarray(pixels_26x26, dtype=np.int16).reshape(26, 26)
        t0 = time.time()
        d = self._predict(img, self._w, self._sage)
        return int(d), (time.time() - t0) * 1000


IP = None
TEST_IMGS = None
TEST_LABS = None
TICK_HZ = 0.0


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a, **k): pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            return self._json(200, {
                "ok": True,
                "mode": "manual" if IP.manual_mode else "auto",
                "tick_hz": TICK_HZ,
                "vision": True,
                "test_images": len(TEST_IMGS) if TEST_IMGS is not None else 0,
            })
        if self.path.startswith("/test_image/"):
            if TEST_IMGS is None:
                return self._json(503, {"error": "start with --images images.mem"})
            try:
                idx = int(self.path.rsplit("/", 1)[-1])
            except ValueError:
                return self._json(400, {"error": "bad index"})
            if not (0 <= idx < len(TEST_IMGS)):
                return self._json(400, {"error": "out of range"})
            pred, ms = IP.classify(TEST_IMGS[idx])
            r = {"prediction": pred, "elapsed_ms": ms}
            if TEST_LABS is not None:
                r["ground_truth"] = int(TEST_LABS[idx])
            return self._json(200, r)
        return self._json(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(n)

        # --- drone: identical contract to drone_server.py ---
        if self.path == "/tick":
            try:
                req = json.loads(body)
                return self._json(200, IP.tick(req["q"], req["omega"], req["thrust"]))
            except Exception as e:
                return self._json(500, {"error": str(e)})

        # --- debug fault injection + self-heal toggle (DEBUG bitstream) ---
        if self.path == "/fault":
            try:
                req = json.loads(body) if body else {}
                state = IP.set_fault(
                    enable=req.get("enable", 0),
                    pe=req.get("pe", 0),
                    mode=req.get("mode", 0),
                    sage_en=req.get("sage_en", 0),
                )
                return self._json(200, state)
            except Exception as e:
                return self._json(500, {"error": str(e)})

        # --- vision ---
        if self.path == "/infer_raw":
            if len(body) != 676 * 2:
                return self._json(400, {"error": f"need {676*2} bytes"})
            pix = np.frombuffer(body, dtype="<i2").reshape(26, 26)
            try:
                pred, ms = IP.classify(pix)
            except Exception as e:
                return self._json(500, {"error": str(e)})
            return self._json(200, {"prediction": pred, "elapsed_ms": ms})

        if self.path == "/infer":
            try:
                pix = np.array(json.loads(body)["pixels"], dtype=np.int16).reshape(26, 26)
            except Exception as e:
                return self._json(400, {"error": str(e)})
            pred, ms = IP.classify(pix)
            return self._json(200, {"prediction": pred, "elapsed_ms": ms})

        return self._json(404, {"error": "not found"})


def _load_mem_int16(path):
    with open(path) as f:
        vals = [int(l.strip(), 16) for l in f if l.strip()]
    return np.array(vals, dtype=np.uint16).view(np.int16)


def main():
    global IP, TEST_IMGS, TEST_LABS, TICK_HZ
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9100)   # same default as drone_server
    ap.add_argument("--bitfile", default="sage16_shared.bit")
    ap.add_argument("--manual", action="store_true")
    ap.add_argument("--tick-hz", type=float, default=1000.0)
    ap.add_argument("--clk-hz", type=float, default=100e6)
    ap.add_argument("--images", default=None)
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--weights-dir", default=".")
    args = ap.parse_args()

    if args.mock:
        IP = MockIP(args.weights_dir)
        print("MOCK mode (CPU, no board)")
    else:
        from pynq import Overlay
        ov = Overlay(args.bitfile)
        IP = SharedIP(ov.drone_0)     # IP instance name from the block design
        IP.manual_mode = args.manual
        print(f"Overlay loaded: {args.bitfile}")
        if args.manual:
            IP.disarm_autotick()
            print("Control mode: MANUAL (software pulses each tick)")
        else:
            TICK_HZ = IP.arm_autotick(args.clk_hz, args.tick_hz)
            print("Control mode: AUTO (fabric self-ticks)")

    if args.images:
        import os
        TEST_IMGS = _load_mem_int16(args.images).reshape(-1, 26, 26)
        print(f"Loaded {len(TEST_IMGS)} test images")
        lp = os.path.join(os.path.dirname(os.path.abspath(args.images)), "labels.mem")
        if os.path.exists(lp):
            TEST_LABS = _load_mem_int16(lp)

    print(f"Server: http://{args.host}:{args.port}")
    print("  POST /tick  (drone)   POST /infer_raw  (vision)")
    HTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
