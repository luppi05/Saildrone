import numpy as np
from saildrone_hydro import get_vals
import my_functions as mf
#constants
mass = 2500.0
I = 10000.0
#predefined waypoints
waypoints = [
    (0, 175),
    (100, 350), 
    (150, 275),
    (200, 350),
    (15, -50),
    (0,0),
    (-500, 200)
]

current_wp_index = 0
#returns optimal sail and rudder angle for control inputs
def control_inputs_auto(t, x, y, theta, vx, vy, omega):
    true_wind = mf.wind_vector(t, x, y)
    global current_wp_index
    current_wp_index = mf.pick_current_waypoint(x, y, waypoints, current_wp_index)
    #just continue on current path if there are no more waypoints
    if current_wp_index >= len(waypoints):
        theta_target = theta
        beta_rudder = mf.rudder_controller(theta, theta_target, omega)
        beta_sail = mf.sail_controller_vmg(theta, vx, vy, true_wind)
        return beta_sail, beta_rudder
    

    wp = waypoints[current_wp_index] #set target waypoint
    theta_target = mf.heading_to_waypoint(x, y, wp) #calculate target heading
    #tack into the wind if the heading is upwind
    #one tack was chosen as opposed to multiple as it is faster
    if abs(theta_target) < np.deg2rad(45):
        theta_target = np.deg2rad(45)

    beta_rudder = mf.rudder_controller(theta, theta_target, omega)
    beta_sail = mf.sail_controller_vmg(theta, vx, vy, true_wind)

    return beta_sail, beta_rudder

def dynamics_auto(t, state):
    x, y, theta, vx, vy, omega = state
#calculate apparent wind
    true_wind = mf.wind_vector(t, x, y)
    v_aw = mf.apparent_wind(vx, vy, true_wind)
    beta_aw = mf.apparent_wind_angle(v_aw)
#get control inputs
    beta_sail, beta_rudder = control_inputs_auto(t, x, y, theta, vx, vy, omega)
    #calculate angles of attack
    theta_sail = theta + beta_sail
    alpha = theta_sail - beta_aw
    #calculate sail forces
    F_aero = mf.aero_force_world(alpha, v_aw)
    M_sail = mf.sail_torque(F_aero, theta)
    #pull hydrodynamic forces
    F_hydro, M_hydro = get_vals(np.array([vx, vy]), theta, omega, beta_rudder)
#sum forces and torques
    Fx = F_aero[0] + F_hydro[0]
    Fy = F_aero[1] + F_hydro[1]
    M_total = M_sail + M_hydro
#calculate accelerations
    ax = Fx / mass
    ay = Fy / mass
    omega_dot = M_total / I
#effects of current
    current = mf.current_vector(t, x, y)
    vx,vy = np.array([vx, vy]) - current

#return state derivatives
    return np.array([
        vx,
        vy,
        omega,
        ax,
        ay,
        omega_dot
    ])
