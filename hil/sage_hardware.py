"""
sage_hardware.py -- drop-in replacement for sage_math.control_tick()
that calls the FPGA on the PYNQ board over HTTP instead of computing
locally.

To use, edit drone_demo.py: change

    from sage_math import control_tick

to

    from sage_hardware import control_tick

and set the SERVER URL below to your PYNQ's IP.

The function signature and return shape are identical to sage_math's, so
the rest of the simulator is unchanged.
"""
import os
import requests


SERVER = os.environ.get("DRONE_SERVER", "http://192.168.137.168:9100")

_session = requests.Session()


def control_tick(q, omega, thrust):
    """Send one control iteration to the FPGA.

    q: 4-tuple of int16 (w, x, y, z)
    omega: 3-tuple of int16
    thrust: int16

    Returns dict with the same shape sage_math.control_tick returns:
        {"error": (w, x, y, z), "torque": (x, y, z), "motors": (m0..m3)}

    Note: the FPGA only reports err.x/y/z (not err.w), so we substitute
    a zero for err.w to keep the dashboard happy. (err.w isn't used by
    the controller anyway.)
    """
    try:
        r = _session.post(
            SERVER + "/tick",
            json={"q": list(q), "omega": list(omega), "thrust": int(thrust)},
            timeout=2.0,
        )
        r.raise_for_status()
        data = r.json()
    except Exception as e:
        # If the server is unreachable, return zeros so the demo doesn't
        # crash. The dashboard's roll/pitch will free-fall and you'll
        # know to check the connection.
        print(f"[sage_hardware] server error: {e}")
        return {"error": (0, 0, 0, 0), "torque": (0, 0, 0),
                "motors": (0, 0, 0, 0)}

    err_x, err_y, err_z = data["error"]
    tx, ty, tz = data["torque"]
    m0, m1, m2, m3 = data["motors"]

    return {
        "error":  (0, err_x, err_y, err_z),    # err.w not reported; not used
        "torque": (tx, ty, tz),
        "motors": (m0, m1, m2, m3),
    }


# Re-export the saturation helper / constants that drone_demo.py might
# pull from sage_math, so a one-line import swap works.
INT16_MIN, INT16_MAX = -(1 << 15), (1 << 15) - 1


def sat_int16(x):
    return max(INT16_MIN, min(INT16_MAX, int(x)))


def to_int16(x):
    x = int(x) & 0xFFFF
    return x - 0x10000 if x & 0x8000 else x
