# SAGE-16 — Shared-Fabric Drone Brain with In-Flight Self-Healing

Host-free MNIST CNN inference **and** a self-stabilizing quadrotor attitude loop, both
running on **one** 4×4 MAC fabric (16 PEs), time-shared, on a PYNQ-Z1 (`xc7z020clg400-1`).
The PL sequences both workloads itself and switches kernel modes per call with **no bitstream
reload** — the shared-fabric claim, demonstrated on silicon. A separate debug build adds a
**fault-in-flight self-heal demo**: a PE is deliberately corrupted mid-flight and the fabric's
ABFT checksum repairs the result on-chip, so the drone stays on setpoint.

**Status — verified on hardware:**
- Vision: test image 0 → digit 7 (~25 ms), bit-exact against `golden.txt`.
- Attitude: `q=(100,30,0,0)` → `err_x=-3000`, `torque_x=-18000`; drone stabilizes in the loop.
- Shared fabric: both workloads on one fabric, control high-priority, no loop starvation.
- Self-heal (debug build): inject PE0 stuck-at-0 → motor0 drops 1000→0 with repair off, and
  holds at 1000 with repair on — same injected fault, `sage_en` the only difference.

---

## What runs where

- **Board (PYNQ PL / fabric):** the whole controller and CNN sequencer. An on-chip tick
  generator paces the control loop (~1 kHz) with nothing external in the timing path. Each
  tick snapshots inputs, then the sequencer runs `attitude_ctrl` (quaternion mode → error +
  PD torque) then `motor_mixer` (matmul mode → 4 motors), arbitrated onto the single 16-PE
  fabric by the scheduler, which switches quat↔matmul modes on-chip. The vision sequencer
  drives the same fabric ports, time-shared, at lower priority so the flight loop never starves.
- **Board (PYNQ PS / ARM):** `shared_server.py` — a pure I/O shim. Loads the overlay, arms
  the on-chip tick once at startup, then serves HTTP: `/tick` (attitude control) and
  `/infer_raw` `/infer` `/test_image` (vision). No control math, no sequencing. One process
  owns the overlay and serves both — only one process may hold the fabric at a time.
- **Laptop:** `drone_vision_demo.py` — a pygame quadrotor you fly with the keyboard while the
  fabric stabilizes it, plus a live webcam panel that classifies a held-up digit on the same
  fabric. Runs the physics, pilot input, sensor/actuator models, and rendering only — no
  control math. Needs `physics.py`, `sage_hardware.py`, `sage_math.py`, `preprocess.py`
  alongside it.

## Quick start

Board:
```
python3 hil/shared_server.py --bitfile sage16_shared.bit --images data/test/images.mem
```
Laptop:
```
python3 hil/drone_vision_demo.py --hardware --server http://<board-ip>:9100
```
Controls: `I/K` pitch · `J/L` roll · `A/D` yaw · `SPACE` toggle pilot · `C` classify webcam
digit · `V` reopen camera · `R` reset · `Q` quit. Use `--software` on the laptop to run the
whole loop without hardware. `GET /health` reports tick mode and rate.

## Build

Consolidated single-tree layout: all `.v` in `rtl/`, weight ROMs in `data/weights/`. From a
Vivado Tcl console:
```
set src_dir     {C:/sage16_integrated/rtl}
set drone_wrap  $src_dir
set sage16_src  $src_dir
set weights_dir {C:/sage16_integrated/data/weights}
cd {C:/sage16_integrated}
source {C:/sage16_integrated/rtl/build_shared_pynq.tcl}
```
Produces `design_1_wrapper.bit` / `design_1.hwh`; rename to `sage16_shared.bit` / `.hwh` and
copy both to the board.

> **Build rule (important):** the Tcl pulls partner fabric files via an explicit **allow-list**
> — it does **not** glob the whole partner folder. That folder has held an old parallel
> codebase (`fake_quat.v`, old `control_top.v`, etc.) whose duplicate module names shadow this
> design and silently break it if globbed in. After any build, confirm the log does **not** say
> `[IP_Flow 19-3833] Unreferenced ... attitude_ctrl.v`. See `HANDOFF.md` for the full story.

## Repository layout

```
rtl/            all Verilog — sequencers, AXI wrapper, and the partner fabric + reliability files
hil/            board-side server + laptop-side demo/sim (Python)
data/weights/   CNN weight ROMs (committed — the build reads these)
data/test/      test vectors: images.mem, labels.mem, img*.mem, golden.txt (gitignored, regenerable)
constraints/    PYNQ-Z1 pin constraints
build_shared_pynq.tcl   Vivado build: IP packaging + block design + bitstream
```

### Key RTL
- `sage16_shared_axi.v` — AXI4-Lite wrapper; register map `0x000–0x3FF`.
- `sage16_scheduler.v` — 3-way fabric arbiter (attitude/mixer/vision), control priority over
  vision; includes the ungated stale-`done` race fix.
- `control_top_shared.v` — host-free control top (on-chip self-tick), vision lane exposed,
  fault-injection + `sage_en` debug registers.
- `vision_top.v` — host-free CNN sequencer.
- `sage16_vision_soc.v` — SoC top: wires control + vision to one scheduler/fabric.
- `attitude_ctrl.v`, `motor_mixer_sage16.v` — PD attitude controller + quad-X mixer.
- `sram_1rw_256x32.v` — behavioral BRAM model (FPGA path; the partner SRAM macro is the ASIC path).
- `sage16_top.v` — fabric-side wrapper: kernel sequencing over `sage16_4x4_mac`, plus the
  debug fault-injection mux and ABFT erasure-repair logic (debug build only).
- Fabric (partner): `sage16_4x4_mac.v`, `pe.v`, `mod3_reduce.v`, and the reliability
  companions (`abft_checksum.v` wired for the self-heal demo; others present in-tree).

## Register map
```
q_w/x/y/z    0x000/4/8/C     motors     0x100/4/8/C    CTRL      0x200
omega x/y/z  0x010/14/18     torque     0x110/14/18    STATUS    0x204
thrust       0x01C           err x/y/z   0x11C/120/124  AUTO_EN  0x208
                                                        TICK_DIV 0x20C

VIS_PIX     0x300  ([25:16]=idx, [15:0]=int16)
VIS_CTRL    0x304  (bit0 = classify)
VIS_STATUS  0x308  (bit0 = busy, bit1 = valid, [5:2] = digit)

-- debug build only (fault-in-flight self-heal) --
DBG_FAULT_EN    0x210  (bit0 = inject)
DBG_FAULT_PE    0x214  ([3:0] = which PE, 0..15)
DBG_FAULT_MODE  0x218  ([1:0] = 0 stuck-at-0, 1 stuck-at-1, 2 bit-28 flip)
SAGE_EN         0x21C  (bit0 = 1 → ABFT-repair the flagged PE; 0 → pass through)

STATUS = {8'd0, mm_calls[7:0], ac_calls[7:0], 5'd0, tick_busy, mm_done, ac_done}
```
Motor lanes map to PEs 0/4/8/12 (column 0 of the array): PE0→motor0, PE4→motor1, etc.

## Fault-in-flight self-heal (debug build)

The **production** datapath is clean — no injection hook. A **separate debug bitstream** adds
a PS-writable fault-injection register and a synthesizable corruption mux around
`sage16_4x4_mac`, plus on-fabric ABFT erasure repair gated by `sage_en`. `abft_checksum`
independently computes each row's true checksum from the input rails (immune to a PE fault);
the repair subtracts the observed-vs-checksum error out of the flagged PE. One subtraction per
flagged PE, host-free. Demo (Jupyter):
```python
from pynq import Overlay
ol = Overlay("sage16_debug.bit"); ctrl = ol.drone_0
w = lambda a,v: ctrl.write(a,v); r = lambda a: ctrl.read(a)
w(0x20C,100_000); w(0x208,1)                 # arm on-chip tick
m = lambda: [r(0x100),r(0x104),r(0x108),r(0x10C)]
print("clean          :", m())               # [1000,1000,1000,1000]
w(0x214,0); w(0x218,0); w(0x21C,0); w(0x210,1)
print("fault, SAGE off:", m())               # motor0 -> 0
w(0x21C,1)
print("fault, SAGE on :", m())               # motor0 -> 1000 (healed)
w(0x210,0); w(0x21C,0)
```

## Conventions
- **Quaternion scale: identity = `(100,0,0,0)`.** Controller target is hard-coded to this.
  Feeding a 16384-scaled quaternion silently zeroes the control math.
- `KP=6, KD=2`; omega scale 1000 counts/rad/s.
- Loop rate = `f_clk / (TICK_DIV+1)`; control (~1 kHz, fabric) and laptop frame rate (~60 Hz)
  are independent.
- CNN: 26×26 → conv1(3×3, 1→8) → pool4 → conv2(3×3, 8→16) → `>>3` → FC1(256→32)+ReLU →
  FC2(32→10) → argmax. Bit-exact against `golden.txt` (img0→7 …). One ReLU, between FC1 and FC2.

## Notes / gotchas
- **One process may hold the overlay.** Don't run a Jupyter `Overlay()` cell while the server
  runs; restart the server after any `Overlay()` call. Always re-copy the `.bit` after a rebuild
  and restart the kernel before reloading the overlay.
- Server defaults to AUTO self-tick; the host is not in the control loop's timing path. The
  auto-tick requires this bitstream — on an older bit, arming does nothing (use `--manual`).
- PYNQ result registers are int16 in 32-bit words; mask to low 16 bits (`raw & 0xFFFF`) where
  relevant. Device detection only works from the board's Python environment.
- This is a **PYNQ-Z1** demo vehicle. The fabric + self-heal logic is platform-agnostic
  Verilog; the AXI wrapper, pin constraints, and bitstream are board-specific.

## Future work
- Time-series flight A/B (control-error vs time, SAGE off vs on, T₀ marked) as the paper figure;
  the register-level A/B above is the mechanism proof.
- Wire the remaining reliability companions (locate/repair beyond single-fault, self-validating
  reduction for the vision GAP/pool).
- Smarter matmul tiling (the current scheme replicates the input row, wasting ~75% of each 4×4
  GEMM) and batched AXI transfers toward microsecond-scale latency.
- IMU (MPU6050) over PL AXI-IIC is detected at `0x68` but not yet wired into the loop.

