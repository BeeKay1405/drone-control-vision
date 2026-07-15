"""
physics.py — minimal quadrotor rigid-body simulator.

Internal state is kept in SI units (radians, rad/s, normalized quaternion
components in [-1, 1]). We convert to/from the SAGE int16 fixed-point
format only at the boundary where the controller is called.

Why this split: the controller (real hardware or its Python mirror) speaks
fixed-point. Physics is easier to reason about in floats. Conversion is
explicit so we know exactly where the precision break is and can match
it bit-for-bit when we move to hardware.

The dynamics are a deliberately simple quadrotor model:
  - rigid body, principal-axis inertia (Ix, Iy, Iz)
  - motor commands are linearly mapped to thrust and body torques via
    the quad-X mixer (so the controller can drive them directly)
  - gravity, drag (linear angular damping), and external disturbance
    torques can be added but aren't required for attitude-hold
  - the drone is attitude-only (we don't track position); pilot inputs
    add target roll/pitch/yaw rates that the physics integrates as
    user demands, NOT as "where the drone is"
"""

import math
import numpy as np


# ----- physical constants --------------------------------------------------
# Tuned so the motion looks plausible at the demo's update rate (~60 Hz).
# These are NOT meant to be realistic; they're chosen so user inputs feel
# responsive and the controller has something to correct.

I_BODY = np.array([0.02, 0.02, 0.04])   # principal-axis inertia (kg m^2)
ANG_DRAG = 0.05                          # linear angular drag (N m s / rad)
MOTOR_TORQUE_GAIN = 5e-5                 # converts motor count -> body torque
DT_DEFAULT = 1.0 / 60.0


# ----- quaternion helpers --------------------------------------------------

def quat_normalize(q):
    n = math.sqrt(sum(c*c for c in q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def quat_mul(a, b):
    """Hamilton product, float version."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def quat_from_axis_angle(axis, angle):
    """axis should be unit. angle in radians."""
    h = angle * 0.5
    s = math.sin(h)
    return (math.cos(h), axis[0]*s, axis[1]*s, axis[2]*s)


def quat_to_euler(q):
    """Returns (roll, pitch, yaw) in radians (ZYX intrinsic)."""
    w, x, y, z = q
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ----- the drone state ----------------------------------------------------

class Drone:
    """A single quadrotor's attitude state."""

    def __init__(self):
        # Orientation (world->body): identity = upright
        self.q = (1.0, 0.0, 0.0, 0.0)
        # Body-frame angular velocity (rad/s)
        self.omega = np.zeros(3)
        # Pilot's commanded angular velocity (rad/s). The controller's
        # job is to make self.omega track this. For a simple "attitude
        # hold" demo, the pilot input is fed via the omega-bias hack
        # below: pretend the drone is at target_omega, the controller
        # tries to make actual_omega = 0, the net effect is the drone
        # rotates as the pilot wants.
        #
        # Simpler approach we actually take: pilot inputs add an
        # external disturbance torque, the controller fights it, and
        # if the pilot lets go the drone returns to level. This
        # behaves more like a real fly-by-wire system.
        self.pilot_torque = np.zeros(3)  # added each step
        # External disturbance (wind, etc.) — for the fault-injection
        # part later
        self.disturbance_torque = np.zeros(3)

    # --- conversion to SAGE int16 fixed-point ---
    Q_SCALE = 100        # matches RTL: identity = 100
    OMEGA_SCALE = 1000   # rad/s -> int16; 1 rad/s ≈ 1000 counts

    def get_sage_inputs(self):
        """Return (q_int16_tuple, omega_int16_tuple) for the controller."""
        from sage_math import to_int16, sat_int16
        q = self.q
        qi = tuple(to_int16(int(round(c * self.Q_SCALE))) for c in q)
        oi = tuple(sat_int16(int(round(w * self.OMEGA_SCALE))) for w in self.omega)
        return qi, oi

    # --- apply motor commands & advance time ---
    def step(self, motors, dt=DT_DEFAULT):
        """Apply motor commands (4-tuple of ints from motor_mixer) and
        advance physics by dt seconds.

        The mixer matrix in motor_mixer_sage16.v is invertible (the
        standard quad-X mixer). To recover the body-frame torques the
        motors actually produce, we apply the inverse mapping. But for
        simulation we can do it more simply: because the mixer is
        [thrust;tau_x;tau_y;tau_z] -> motor counts, and the controller
        already computed those torques, we can either:
          (a) apply the controller's torques directly to the body, or
          (b) apply the inverse-mixed motor torques (which agrees with
              the controller in healthy state, but will differ if a
              motor is broken / saturated).
        We do (b) so that motor faults later in the project (or motor
        saturation) actually affect the physics.
        """
        from sage_math import A_MIX
        m = np.array(motors, dtype=np.float64)
        # Invert the mixer to back out the equivalent torques. The
        # quad-X mixer matrix is its own scaled inverse (orthogonal up to
        # a factor of 4), so A_inv = A_mix.T / 4.
        A_inv = A_MIX.T.astype(np.float64) / 4.0
        eff = A_inv @ m                        # [thrust, tau_x, tau_y, tau_z]
        body_torque = eff[1:] * MOTOR_TORQUE_GAIN

        # Total torque on the body
        torque = body_torque + self.pilot_torque + self.disturbance_torque
        # Newton-Euler on angular velocity, ignoring cross-coupling terms
        # (good enough for small-angle attitude hold; full equation would
        # include omega x (I omega))
        omega_dot = (torque - ANG_DRAG * self.omega) / I_BODY
        self.omega = self.omega + omega_dot * dt

        # Integrate orientation using quaternion derivative.
        # q_dot = 0.5 * q ⊗ (0, omega)
        w, x, y, z = self.q
        ox, oy, oz = self.omega
        dq = (
            0.5 * (-x*ox - y*oy - z*oz),
            0.5 * ( w*ox + y*oz - z*oy),
            0.5 * ( w*oy - x*oz + z*ox),
            0.5 * ( w*oz + x*oy - y*ox),
        )
        self.q = quat_normalize(tuple(c + d*dt for c, d in zip(self.q, dq)))

    def reset(self):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.omega = np.zeros(3)
        self.pilot_torque = np.zeros(3)
        self.disturbance_torque = np.zeros(3)
