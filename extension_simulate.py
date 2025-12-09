import numpy as np
import matplotlib.pyplot as plt
from ode import solve_ivp
from extension_programme import dynamics_auto
from extension_programme import waypoints, wind_history, current_history


wps = np.array(waypoints)


#simulate from 0s to 450s with initial conditions
def simulate_auto():

    t0 = 0
    tmax = 500
    dt = 0.05

    z0 = np.array([
        0.0, #initial x position
        0.0, #initial y position
        np.pi/2, #initial heading angle
        0.0, #initial x velocity
        2.9, #initial y velocity
        0.0 #initial angular velocity
    ])
#solve ODE
    t, z = solve_ivp(dynamics_auto, t0, tmax, dt, z0, method='RK')
    wind_hist = np.array(wind_history)
    current_hist = np.array(current_history)
    return t, z, wind_hist, current_hist

#plots trajectory of drone
def plot_trajectory(t, z):
    x = z[0]
    y = z[1] 

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, 'b', linewidth=2)
    plt.scatter(wps[:,0], wps[:,1], c='red', s=50, marker='o', zorder=10)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
#plots wind history
def plot_wind(wind_hist):
    t = np.linspace(0, 500, len(wind_hist[:,0]))

    plt.figure(figsize=(9,5))
    plt.plot(t, wind_hist[:,0], label="Wind velocity x-direction (m/s)")
    plt.plot(t, wind_hist[:,1], label="Wind velocity y-direction(m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Wind velocity (m/s)")
    plt.grid(True)
    plt.title("Wind velocity at drone's position over time")
    plt.legend()
    plt.show()
#plots current history
def plot_current(current_hist):
    t = np.linspace(0, 500, len(current_hist[:,0]))

    plt.figure(figsize=(9,5))
    plt.plot(t, current_hist[:,0], label="Current velocity x-direction (m/s)")
    plt.plot(t, current_hist[:,1], label="Current velocity y-direction (m/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Current velocity (m/s)")
    plt.grid(True)
    plt.title("Current velocity at drone's position over time")
    plt.legend()
    plt.show()

#main function to run simulation and plot
if __name__ == "__main__":
    t, z, wind_hist, current_hist = simulate_auto()
    plot_trajectory(t, z)
    plot_wind(wind_hist)
    plot_current(current_hist)
