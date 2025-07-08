"""
ankle_kinematics.py  :  fast closed-chain ankle kinematics
----------------------------------------------------------
roll  = theta_5   (rad, limited to ±20°)
pitch = theta_6   (rad, limited to ±20°)
--------------
left_leg_motor_angles (roll, pitch)      -> (theta_m5, theta_m6)
right_leg_motor_angles(roll, pitch)      -> (theta_m5, theta_m6)
left_leg_foot_angles  (theta_m5, theta_m6, guess)  -> (roll, pitch)
right_leg_foot_angles (theta_m5, theta_m6, guess)  -> (roll, pitch)
"""
import numpy as np

# ──────────────────────────────────────────────────────────────────────
# 1.  CAD geometry (extracted once ->  NumPy arrays)
# ──────────────────────────────────────────────────────────────────────
def _geom(px0, py0, pz0, d, a, h, l, s):
    return dict(Px0=np.array(px0, dtype=float),
                Py0=np.array(py0, dtype=float),
                Pz0=np.array(pz0, dtype=float),
                d  =np.array(d,  dtype=float),
                a  =np.array(a,  dtype=float),
                h  =np.array(h,  dtype=float),
                l  =np.array(l,  dtype=float),
                s  =np.array(s,  dtype=float))

LEFT  = _geom(px0=[0.043, 0.043],
              py0=[ 0.023, -0.023],
              pz0=[-0.020, -0.020],
              d  =[0.043, 0.043],
              a  =[0.023, 0.023],
              h  =[0.230, 0.145],
              l  =[0.25, 0.165],
              s  =[ 1, -1])

RIGHT = _geom(px0=[0.043, 0.043],
              py0=[ 0.023, -0.023],
              pz0=[-0.020, -0.020],
              d  =[0.043, 0.043],
              a  =[0.023, 0.023],
              h  =[0.145, 0.230],
              l  =[0.165, 0.25],
              s  =[ 1, -1])

_LOWER = -0.3490658402442932   # −20 deg
_UPPER =  0.3490658402442932   # +20 deg


# ──────────────────────────────────────────────────────────────────────
# 2.  Helper – rotate both reference points (vectorized)
# ──────────────────────────────────────────────────────────────────────
def _rotate(c5, s5, c6, s6, Px0, Py0, Pz0):
    Px =  c6*Px0 + s6*Pz0
    Py =  s5*s6*Px0 + c5*Py0 - s5*c6*Pz0
    Pz = -c5*s6*Px0 + s5*Py0 + c5*c6*Pz0
    return Px, Py, Pz


# ──────────────────────────────────────────────────────────────────────
# 3.  Inverse map  (roll, pitch) -> (theta_m5, theta_m6)
# ──────────────────────────────────────────────────────────────────────
def _ankle_mapping(roll: float, pitch: float, G):
    Px0, Py0, Pz0 = G['Px0'], G['Py0'], G['Pz0']
    d, a, h, l, sdir = G['d'], G['a'], G['h'], G['l'], G['s']

    c5, s5 = np.cos(roll),  np.sin(roll)
    c6, s6 = np.cos(pitch), np.sin(pitch)

    Px, Py, Pz = _rotate(c5, s5, c6, s6, Px0, Py0, Pz0)

    z = Pz - h
    x = d  - Px

    u = 2.0*a*Py
    v = 2.0*a*z
    w = (a*a + x*x + Py*Py + z*z - l*l) * sdir

    gamma = np.arctan2(v, u)
    R     = np.hypot(u, v)
    return gamma + np.arccos(w / R)


def left_leg_motor_angles(roll: float, pitch: float):
    """(roll, pitch) -> (theta_m5, theta_m6) for the LEFT ankle."""
    return _ankle_mapping(roll, pitch, LEFT)


def right_leg_motor_angles(roll: float, pitch: float):
    """(roll, pitch) -> (theta_m5, theta_m6) for the RIGHT ankle."""
    return _ankle_mapping(roll, pitch, RIGHT)[::-1]


# ──────────────────────────────────────────────────────────────────────
# 4.  Forward map  (theta_m5, theta_m6) -> (roll, pitch) : x-step Newton
# ──────────────────────────────────────────────────────────────────────
def _forward_one_leg(thetam, guess, G):
    Px0, Py0, Pz0 = G['Px0'], G['Py0'], G['Pz0']
    d, a, h, l, sdir = G['d'], G['a'], G['h'], G['l'], G['s']

    c_m, s_m = np.cos(thetam), np.sin(thetam)  # constant during solve
    roll, pitch = guess

    num_iter = 5
    for _ in range(num_iter):                         # 3 Newton iterations are enough
        c5, s5 = np.cos(roll),  np.sin(roll)
        c6, s6 = np.cos(pitch), np.sin(pitch)

        Px, Py, Pz = _rotate(c5, s5, c6, s6, Px0, Py0, Pz0)

        z  = Pz - h
        x  = d  - Px
        yb = Py

        W = (a*a + x*x + yb*yb + z*z - l*l) / (2.0*a)
        f = yb*c_m + z*s_m - sdir*W            # residual (length-2)

        # analytic Jacobian --------------------------------------------------
        # partials of reference-point coordinates
        dPx_dr = np.zeros(2)                   # Px independent of roll
        dPy_dr =  c5*s6*Px0 - s5*Py0 - c5*c6*Pz0
        dPz_dr = s5*s6*Px0 + c5*Py0 - s5*c6*Pz0

        dPx_dp = -s6*Px0 + c6*Pz0
        dPy_dp =  s5*c6*Px0 + s5*s6*Pz0
        dPz_dp = -c5*c6*Px0 - c5*s6*Pz0

        dz_dr, dz_dp = dPz_dr, dPz_dp
        dy_dr, dy_dp = dPy_dr, dPy_dp
        dx_dr, dx_dp = -dPx_dr, -dPx_dp       # because x = d - Px

        dW_dr = (x*dx_dr + yb*dy_dr + z*dz_dr) / a
        dW_dp = (x*dx_dp + yb*dy_dp + z*dz_dp) / a

        J11 = dy_dr*c_m + dz_dr*s_m - sdir*dW_dr
        J12 = dy_dp*c_m + dz_dp*s_m - sdir*dW_dp

        # 2x2 Jacobian
        det = J11[0]*J12[1] - J12[0]*J11[1]
        if abs(det) < 1e-12:
            break                             # singular -> keep last iterate

        # solve J \delta = f   (manual 2×2 inverse)
        dx =  ( f[0]*J12[1] - f[1]*J12[0]) / det
        dy = (-f[0]*J11[1] + f[1]*J11[0]) / det
        roll  -= dx
        pitch -= dy

        # converge?
        if abs(dx) < 1e-12 and abs(dy) < 1e-12:
            break

    # clip to joint limits
    roll  = np.clip(roll,  _LOWER, _UPPER)
    pitch = np.clip(pitch, _LOWER, _UPPER)
    return roll, pitch


def left_leg_foot_angles(theta_m5, theta_m6,
                         roll0: float = 0.0, pitch0: float = 0.0):
    """(theta_m5, theta_m6) -> (roll, pitch)  for the LEFT ankle."""
    return _forward_one_leg(np.array([theta_m5, theta_m6]),
                            (roll0, pitch0), LEFT)


def right_leg_foot_angles(theta_m5, theta_m6,
                          roll0: float = 0.0, pitch0: float = 0.0):
    """(theta_m5, theta_m6) -> (roll, pitch)  for the RIGHT ankle."""
    # swap order back before solver
    return _forward_one_leg(np.array([theta_m6, theta_m5]),
                            (roll0, pitch0), RIGHT)


# ──────────────────────────────────────────────────────────────────────
# 6.  Jacobian  J_inv = \partial\thet_m / \partial[roll,pitch]    
#     ––– theta_motor velocities –––
#     theta_m_dot      =  J_inv * [ roll̇ , pitcḣ ]^T
#     [roll̇ , pitcḣ ]ᵀ  =  J_inv^-1 @ theta_m_dot
# ──────────────────────────────────────────────────────────────────────
def _ankle_jacobian_analytic(roll: float, pitch: float, G):
    """
    Return 2x2 Jacobian  J_inv  with entries
           [ \partialtheta_m5/\partial_roll   \partialtheta_m5/\partial_pitch ]
           [ \partialtheta_m6/\partial_roll   \partialtheta_m6/\partial_pitch ]
    derived analytically from the closed-form inverse map.
    """
    Px0, Py0, Pz0 = G['Px0'], G['Py0'], G['Pz0']
    d,  a,  h, l, sdir = G['d'], G['a'], G['h'], G['l'], G['s']

    c5, s5 = np.cos(roll),  np.sin(roll)
    c6, s6 = np.cos(pitch), np.sin(pitch)

    # rotated points and their first derivatives --------------------
    Px =  c6*Px0 + s6*Pz0
    Py =  s5*s6*Px0 + c5*Py0 - s5*c6*Pz0
    Pz = -c5*s6*Px0 + s5*Py0 + c5*c6*Pz0

    # first partials
    dPy_dr =  c5*s6*Px0 - s5*Py0 - c5*c6*Pz0
    dPz_dr = s5*s6*Px0 + c5*Py0 - s5*c6*Pz0

    dPy_dp =  s5*c6*Px0 + s5*s6*Pz0          
    dPz_dp = -c5*c6*Px0 - c5*s6*Pz0
    dPx_dp = -s6*Px0 + c6*Pz0
    dx_dp  = -dPx_dp

    z  = Pz - h
    x  = d  - Px
    y  = Py

    # helpers for each branch (vectorised length-2) -----------------
    u = 2.0*a*y
    v = 2.0*a*z
    w = (a*a + x*x + y*y + z*z - l*l) * sdir
    R = np.hypot(u, v)                       # sqrt(u²+v²)
    s_over = w / R                           # argument of arccos

    # dγ/dq,  γ = atan2(v,u)
    du_dr = 2.0*a*dPy_dr
    dv_dr = 2.0*a*dPz_dr
    du_dp = 2.0*a*dPy_dp
    dv_dp = 2.0*a*dPz_dp

    dgamma_dr = (-v*du_dr + u*dv_dr) / (u*u + v*v)
    dgamma_dp = (-v*du_dp + u*dv_dp) / (u*u + v*v)

    # derivatives of s := w/R
    dw_dr = (2*x*0              # dx/dr = 0
             + 2*y*dPy_dr
             + 2*z*dPz_dr) * sdir
    
    dw_dp = (2*x*dx_dp + 2*y*dPy_dp + 2*z*dPz_dp) * sdir
    dR_dr = (u*du_dr + v*dv_dr) / R
    dR_dp = (u*du_dp + v*dv_dp) / R

    ds_dr = (dw_dr*R - w*dR_dr) / (R*R)
    ds_dp = (dw_dp*R - w*dR_dp) / (R*R)

    # d/dq arccos(s) = − ds / sqrt(1−s^2)
    fac = -1.0 / np.sqrt(np.maximum(1.0e-14, 1.0 - s_over*s_over))
    dacos_dr = fac * ds_dr
    dacos_dp = fac * ds_dp

    # final Jacobian rows
    J_inv = np.empty((2, 2), dtype=float)
    J_inv[0, 0] = dgamma_dr[0] + dacos_dr[0]   # \partial theta_m5/\partial roll
    J_inv[0, 1] = dgamma_dp[0] + dacos_dp[0]   # \partial theta_m5/\partial pitch
    J_inv[1, 0] = dgamma_dr[1] + dacos_dr[1]   # \partial theta_m6/\partial roll
    J_inv[1, 1] = dgamma_dp[1] + dacos_dp[1]   # \partial theta_m6/\partial pitch
    return J_inv

# ───────────────────── finite-difference Jacobian (debug) ───────────
def _ankle_jacobian_fd(roll, pitch, G, h: float = 1.0e-8):
    """Finite-difference """
    f0 = _ankle_mapping(roll, pitch, G)
    f_r = _ankle_mapping(roll + h, pitch, G)
    f_p = _ankle_mapping(roll, pitch + h, G)
    return np.column_stack(((f_r - f0) / h,
                            (f_p - f0) / h))

# pick analytic path (comment next line to force FD)
# ankle_jacobian = _ankle_jacobian_analytic
ankle_jacobian = _ankle_jacobian_fd     #

# Jacobian in **motor order** (swap rows for right leg)
def _J_motor(r, p, *, left=True):
    G = LEFT if left else RIGHT
    J = ankle_jacobian(r, p, G)
    return J if left else J[::-1]

# ───────────────────── velocity & torque maps ───────────────────────
def motor_velocities_from_foot(r, p, r_d, p_d, *, left=True):
    return _J_motor(r, p, left=left) @ np.array([r_d, p_d])

def foot_velocities_from_motor(r, p, theta_m_d, *, left=True):
    return np.linalg.solve(_J_motor(r, p, left=left), theta_m_d)

def ankle_torque_from_motor(r, p, tau_m5, tau_m6, *, left=True):
    return _J_motor(r, p, left=left).T @ np.array([tau_m5, tau_m6])

def motor_torque_from_ankle(r, p, tau_p, tau_r, *, left=True):
    J = _J_motor(r, p, left=left)
    det = J[0,0]*J[1,1] - J[0,1]*J[1,0]
    Jinv = np.array([[ J[1,1], -J[0,1]],
                     [-J[1,0],  J[0,0]]]) / det
    return Jinv.T @ np.array([tau_p, tau_r])

# ──────────────────────────────────────────────────────────────────────
# 5.  Main
# ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    roll_cmd  = np.deg2rad(0.0)
    pitch_cmd = np.deg2rad(19)

    theta_m_left  = left_leg_motor_angles (roll_cmd, pitch_cmd)
    theta_m_right = right_leg_motor_angles(roll_cmd, pitch_cmd)

    est_roll_L,  est_pitch_L  = left_leg_foot_angles (*theta_m_left)
    est_roll_R,  est_pitch_R  = right_leg_foot_angles(*theta_m_right)

    print("Cmd   : ", np.rad2deg([roll_cmd, pitch_cmd]))
    print("theta_motor_left", theta_m_left)
    print("theta_m_right", theta_m_right)
    print("Left  : ", np.rad2deg([est_roll_L, est_pitch_L]))
    print("Right : ", np.rad2deg([est_roll_R, est_pitch_R]))
    
    '''TEST VELOCITY'''
    roll_dot_cmd, pitch_dot_cmd = np.deg2rad([1.0, -2.0])        # 1 deg/s roll, −2 deg/s pitch
    theta_dot = motor_velocities_from_foot(roll_cmd, pitch_cmd, roll_dot_cmd, pitch_dot_cmd, left=True)
    roll_dot_est, pitch_dot_est = foot_velocities_from_motor(est_roll_L, est_pitch_L, theta_dot, left=True)
    print("input foot vel  :", np.rad2deg([roll_dot_cmd, pitch_dot_cmd]), "deg/s")
    print("back-computed   :", np.rad2deg([roll_dot_est, pitch_dot_est]), "deg/s")

    J_fd   = _ankle_jacobian_fd(roll_cmd, pitch_cmd, LEFT)   
    J_anal   = _ankle_jacobian_analytic(roll_cmd, pitch_cmd, LEFT)

    print("max|J_fd - J_anal| =", np.max(np.abs(J_fd - J_anal)))

    tau_m  = np.array([3.0, -4.0])              # Nm at the motors
    tau_q  = ankle_torque_from_motor(roll_cmd, pitch_cmd, *tau_m, left=True)
    tau_m2 = motor_torque_from_ankle(roll_cmd, pitch_cmd, *tau_q, left=True)
    
    print("ankle torques :", tau_q)
    print("back to motor :", tau_m2)            # 

    J_fd  = _ankle_jacobian_fd(roll_cmd, pitch_cmd, LEFT)    # your FD helper
    tau_q_fd = J_fd.T @ tau_m

    print("tau_q_fd = ", tau_q_fd)    # 