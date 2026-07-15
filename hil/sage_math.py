"""
sage_math.py — Python mirror of the SAGE-16 fixed-point control math.

These functions reproduce the exact arithmetic that your Verilog modules
attitude_ctrl.v and motor_mixer_sage16.v compute, with the same integer
overflow / sign-extension behavior. When you later replace these functions
with real hardware calls over AXI-Lite, the rest of the simulator does not
change.

Q-format conventions (matching your RTL):
    * Quaternion components are signed int16 with scale 100 = "1.0".
      Identity = (100, 0, 0, 0). q_target in attitude_ctrl.v is hard-coded
      to (16'sd100, 0, 0, 0).
    * Angular velocity (omega) is signed int16, raw integer units.
      The physics layer is responsible for choosing a scaling such that
      omega stays inside int16 over normal operation.
    * Torque is signed int32 (ACC_W=32).
    * Motor commands are signed int32.
    * KP=6, KD=2 in attitude_ctrl.v.
"""

import numpy as np


# ----- bit-width clamping helpers -----------------------------------------

INT16_MIN, INT16_MAX = -(1 << 15), (1 << 15) - 1
INT32_MIN, INT32_MAX = -(1 << 31), (1 << 31) - 1


def to_int16(x):
    """Wrap-around to signed 16-bit (matches Verilog truncation)."""
    x = int(x) & 0xFFFF
    return x - 0x10000 if x & 0x8000 else x


def to_int32(x):
    """Wrap-around to signed 32-bit (matches Verilog truncation)."""
    x = int(x) & 0xFFFFFFFF
    return x - 0x100000000 if x & 0x80000000 else x


def sat_int16(x):
    """Saturate to signed 16-bit. Used when the physics layer needs to
    push omega into the hardware; we saturate rather than wrap so a
    fast-spinning drone doesn't suddenly look stopped."""
    return max(INT16_MIN, min(INT16_MAX, int(x)))


# ----- quaternion Hamilton product ----------------------------------------
# Mirrors quat_sage16.v: produces q1 ⊗ q2 where both are signed int16.
# Result components are int32 (signed).

def hamilton_q15(q1, q2):
    """q1 ⊗ q2 in the RTL's integer convention.

    q1, q2: tuples/arrays of 4 signed int16 components (w, x, y, z).
    Returns: tuple of 4 signed int32 components.

    This is the exact arithmetic the quat_sage16.v module performs.
    Note: because both inputs are "scaled" (identity = 100), the product
    has units of 100*100 = 10000 representing 1.0. attitude_ctrl.v uses
    these int32 values directly as `err_x/y/z` -- it does NOT rescale
    before the KP multiply.
    """
    w1, x1, y1, z1 = (to_int16(v) for v in q1)
    w2, x2, y2, z2 = (to_int16(v) for v in q2)
    rw = w1*w2 - x1*x2 - y1*y2 - z1*z2
    rx = w1*x2 + x1*w2 + y1*z2 - z1*y2
    ry = w1*y2 - x1*z2 + y1*w2 + z1*x2
    rz = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return to_int32(rw), to_int32(rx), to_int32(ry), to_int32(rz)


# ----- attitude_ctrl: error quaternion + PD law ---------------------------

KP = 6   # matches attitude_ctrl.v parameter
KD = 2   # matches attitude_ctrl.v parameter
Q_TARGET = (100, 0, 0, 0)   # identity (level, no yaw); hard-coded in RTL


def attitude_ctrl(q_w, q_x, q_y, q_z, omega_x, omega_y, omega_z):
    """Python mirror of attitude_ctrl.v.

    Inputs: 7 signed int16 (current orientation quaternion + body-frame
            angular velocity).
    Returns: (torque_x, torque_y, torque_z) as signed int32.

    Internally:
        q_current* = (w, -x, -y, -z)
        q_error    = q_target ⊗ q_current*    (one quat call)
        torque_i   = KP * err_i - KD * omega_i  (i in {x, y, z})

    Note that the q_target = (100, 0, 0, 0) means err_w = 100*q_w
    (large), and err_x = -100*q_x, err_y = -100*q_y, err_z = -100*q_z
    -- so the controller drives toward q_w=100 and the other components
    to zero, which is upright/level/no-yaw.
    """
    # Conjugate current quaternion
    q_conj = (q_w, -q_x, -q_y, -q_z)
    # Error = target ⊗ conjugate
    _err_w, err_x, err_y, err_z = hamilton_q15(Q_TARGET, q_conj)

    # PD law. Values can be large -- Python ints are arbitrary precision
    # but the RTL truncates to 32 bits, so we do the same.
    tx = to_int32(KP * err_x - KD * to_int32(omega_x))
    ty = to_int32(KP * err_y - KD * to_int32(omega_y))
    tz = to_int32(KP * err_z - KD * to_int32(omega_z))
    return tx, ty, tz


# ----- motor_mixer: 4x4 GEMM with the quad-X mixer matrix -----------------

# Mirrors motor_mixer_sage16.v exactly.
#   A = quad-X mixer (4x4)
#   B = [thrust; tau_x; tau_y; tau_z; 0; 0; 0; ...] padded to 4x4
#   m_i = C[i][0]   (only column 0 is real signal; columns 1-3 are zeros)
A_MIX = np.array([
    [ 1,  1,  1, -1],
    [ 1, -1,  1,  1],
    [ 1, -1, -1, -1],
    [ 1,  1, -1,  1],
], dtype=np.int64)


def motor_mixer(thrust, torque_x, torque_y, torque_z):
    """Python mirror of motor_mixer_sage16.v.

    Inputs: 4 signed int16. Note: the RTL's matmul_sage16 takes 16-bit
    operands, so torques computed by attitude_ctrl (32-bit) must be
    clipped/scaled to 16-bit before being fed in. We saturate here to
    match how a real driver would do it.

    Returns: (m0, m1, m2, m3) as signed int32 motor commands.
    """
    t = sat_int16(thrust)
    tx = sat_int16(torque_x)
    ty = sat_int16(torque_y)
    tz = sat_int16(torque_z)

    B0 = np.array([t, tx, ty, tz], dtype=np.int64)
    # 4x4 matmul: C[i][j] = sum_k A[i][k]*B[k][j].
    # B has nonzero only in column 0; we just need C[:,0].
    Cc0 = A_MIX @ B0
    return tuple(to_int32(int(v)) for v in Cc0)


# ----- composite: one full control tick ----------------------------------

def control_tick(q, omega, thrust):
    """One full hardware control step.

    Inputs:
        q     : 4-tuple of int16 (w, x, y, z) — current orientation
        omega : 3-tuple of int16 — body-frame angular velocity
        thrust: int16 — desired thrust value (set by physics/user)
    Returns:
        dict with all intermediate values for the dashboard:
            error    : (err_w, err_x, err_y, err_z) int32
            torque   : (tx, ty, tz) int32
            motors   : (m0, m1, m2, m3) int32
    """
    q_w, q_x, q_y, q_z = q
    ox, oy, oz = omega
    # Recompute error so the dashboard can show it (attitude_ctrl uses
    # it internally and only exits with the torques)
    q_conj = (q_w, -q_x, -q_y, -q_z)
    err = hamilton_q15(Q_TARGET, q_conj)
    tx, ty, tz = attitude_ctrl(q_w, q_x, q_y, q_z, ox, oy, oz)
    m = motor_mixer(thrust, tx, ty, tz)
    return {
        "error":  err,
        "torque": (tx, ty, tz),
        "motors": m,
    }


# ----- self-test ----------------------------------------------------------

if __name__ == "__main__":
    # If perfectly level + still, error.vec should be zero and torque zero.
    r = control_tick((100, 0, 0, 0), (0, 0, 0), 1000)
    print("level + still:")
    print(f"  error = {r['error']}")
    print(f"  torque = {r['torque']}")
    print(f"  motors = {r['motors']}")
    # Tilted in roll: q_x != 0 -> should produce nonzero err_x and tau_x.
    r = control_tick((95, 30, 0, 0), (0, 0, 0), 1000)
    print("\ntilted in roll (q_x=30):")
    print(f"  error = {r['error']}")
    print(f"  torque = {r['torque']}")
    print(f"  motors = {r['motors']}")
