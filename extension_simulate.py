import numpy as np
import matplotlib.pyplot as plt
from ode import solve_ivp
from extension_programme import dynamics_auto
from extension_programme import waypoints

wps = np.array(waypoints)


#simulate from 0s to 650s with initial conditions
def simulate_auto():

    t0 = 0
    tmax = 650
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
    return t, z

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
#main function to run simulation and plot
if __name__ == "__main__":
    t, z = simulate_auto()
    plot_trajectory(t, z)
