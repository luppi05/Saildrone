import numpy as np

#constants
rho = 1.225
A = 15.0

# Global gust state (keeps gusts smooth)
gust_state = np.array([0.0, 0.0])

def wind_vector(t, x, y):
    global gust_state
#base wind
    base = np.array([-6.7, 0.0])

#wind that varies sinusoildally over time and space to give smooth variation
    f1 = 0.02 * t #time varying component
    f2 = 0.005 * t + 0.01 * x #spatial varying component in x direction
    f3 = 0.01 * t + 0.005 * y #spatial varying component in y direction
    #x direction
    wx_smooth = base[0]  + 0.8*np.sin(f1)+ 0.4*np.sin(f2 + 1.3) + 0.2*np.sin(f3 + 0.8)
    #y direction
    wy_smooth = base[1]+ 0.8*np.sin(f1 + 2.1) + 0.4*np.sin(f2 + 0.4) + 0.2*np.sin(f3 + 2.7)

    smooth = np.array([wx_smooth, wy_smooth])

    #gusts of wind (ornstein-uhlenbeck process) see the reference
    dt = 0.05  #time step
    tau = 25.0  #time scale of gusts
    sigma = 0.5  #standard deviation of gusts
    #equation from reference
    gust_state += -(gust_state / tau) * dt + sigma * np.sqrt(dt) * np.random.randn(2)

    gust = gust_state
    return smooth + gust

#calculate current
def current_vector(t, x, y):
    
    base = np.array([-0.5, 0.0]) #base current
    
    c1 = 0.00005 * t #time varying component
    c2 = 0.001 * x + 0.005 * y #spatial varying component

    cx = base[0] + 0.1 * np.sin(c1) + 0.05 * np.sin(c2 + 1.1) #x direction

    cy = base[1] + 0.01 * np.sin(c1 + 2.3) + 0.005 * np.sin(c2 + 0.6) #y direction

    current = np.array([cx, cy])

    return current


#calculate apparent wind
def apparent_wind(vx, vy, v_wind):
    v_boat = np.array([vx, vy])
    return v_wind - v_boat

#calculate apparent wind angle
def apparent_wind_angle(v_aw):
    return np.arctan2(v_aw[1], v_aw[0])

#sail orientation
def sail_orientation(theta_boat, beta_sail):
    return theta_boat + beta_sail

#angle of attack based on relative wind spped
def angle_of_attack(theta_sail, beta_aw):
    return theta_sail - beta_aw

#calculate coefficients of lift and drag
def aero_coefficients(alpha):
    CD = 1 - np.cos(2*alpha)
    CL = 1.5*np.sin(2*alpha + 0.5*np.sin(2*alpha))
    return CL, CD

#calculate lift and drag relative to apparent wind
def aero_forces(alpha, v_aw):
    V = np.linalg.norm(v_aw)
    q = 0.5 * rho * V*V
 
    CL, CD = aero_coefficients(alpha)
    L = CL * q * A
    D = CD * q * A
    return L, D

#calculate x and y forces
def aero_force_world(alpha, v_aw):
    L, D = aero_forces(alpha, v_aw)
#direction vectors    
    drag_dir = v_aw / np.linalg.norm(v_aw)
    lift_dir = np.array ([drag_dir[1], -drag_dir[0]])
#total force    
    F = L * lift_dir + D * drag_dir
    return F

#calculate sail torque
def sail_torque(F_world, theta, d=0.1):
    Fx, Fy = F_world

    # Position of sail relative to centre of mass
    rx = -d * np.cos(theta)
    ry = -d * np.sin(theta)

    #torque calculated
    tau = rx * Fy - ry * Fx
    return tau


#functions for extension task only

#ensure angle is between -pi and pi to avoid errors
def angle_wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

#calculate heading to next waypoint
def heading_to_waypoint(x, y, wp):
    dx = wp[0] - x
    dy = wp[1] - y
    return np.arctan2(dy, dx)

#move to next waypoint once current waypoint has been reached
def pick_current_waypoint(x, y, waypoints, idx, tol=5.0):
    #current waypoint
    wp = waypoints[idx]
    if np.hypot(wp[0] - x, wp[1] - y) < tol:
        idx += 1
    if idx == len(waypoints):
        idx = 0  #loop back to first waypoint
    return idx

#rudder controller
def rudder_controller(theta, theta_target, omega, k_p=0.8, k_d=2.5): #constants chosen to give the desired behaviour
    e = angle_wrap(theta_target - theta)    #calculate heading error
    beta = -((k_p * e) - (k_d * omega))    #damping based on omega to stop oscillations

    return np.clip(beta, np.radians(-30), np.radians(30)) #ensure it stays within the 30 degree limit of the rudder

#find optimal sail angle
def sail_controller_vmg(theta, vx, vy, true_wind):
    v_aw = apparent_wind(vx, vy, true_wind)
    #prevents errors if speed is too low
    speed = np.hypot(vx, vy)
    if speed > 0.3:
        theta_vmg = np.arctan2(vy, vx) #optimises for the direction that the boat is travelling
    else:
        theta_vmg = theta

    vmg_dir = np.array([np.cos(theta_vmg), np.sin(theta_vmg)])
#calculate apparent wind angle
    beta_aw = apparent_wind_angle(v_aw)

    best_beta = 0.0 #initialise the best angle at zero
    best_score = -1e12 #initalise the best vmg at a very negative number

    for beta_sail in np.linspace(-np.deg2rad(90), np.deg2rad(90), 100): #iterate through 100 possible angles to choose optimal one

        theta_sail = theta + beta_sail
        alpha = theta_sail - beta_aw
#calculate aerodynamic force in world frame
        F_world = aero_force_world(alpha, v_aw)
#calculate score based on vmg direction
        score = np.dot(F_world, vmg_dir)
#update best score and angle if current one is better
        if score > best_score:
            best_score = score
            best_beta = beta_sail

    return best_beta 
