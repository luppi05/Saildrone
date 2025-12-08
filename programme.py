import numpy as np
from saildrone_hydro import get_vals
import my_functions as mf

mass = 2500.0
I = 10000.0
#returns sail and rudder angle based on time
def control_inputs(t):
    if t < 60:
        beta_sail = np.deg2rad(-45)
        beta_rudder = 0.0
    elif t < 65:
        beta_sail = np.deg2rad(-22.5)
        beta_rudder = np.deg2rad(2.1)
    else:
        beta_sail = np.deg2rad(-22.5)
        beta_rudder = 0.0
        
    return beta_sail, beta_rudder
#state derivative function
def dynamics(t, state):
    x, y, theta, vx, vy, omega = state
#calculate apparent wind
    v_aw = mf.apparent_wind(vx, vy)
    beta_aw = mf.apparent_wind_angle(v_aw)

    beta_sail, beta_rudder = control_inputs(t)
#calculate inputs
    theta_sail = theta + beta_sail
#calculate from relative wind
    alpha = theta_sail - beta_aw
#calculate aerodynamic forces and moments
    F_aero = mf.aero_force_world(alpha, v_aw)
    M_sail = mf.sail_torque(F_aero, theta)

    F_hydro, M_hydro = get_vals(
        np.array([vx, vy]), theta, omega, beta_rudder
    )
#sum forces and calculate accelerations
    Fx = F_aero[0] + F_hydro[0]
    Fy = F_aero[1] + F_hydro[1]
    M_total = M_sail + M_hydro

    ax = Fx / mass
    ay = Fy / mass
    omega_dot = M_total / I
#return state derivatives
    return np.array([
        vx,
        vy,
        omega,
        ax,
        ay,
        omega_dot
    ])
