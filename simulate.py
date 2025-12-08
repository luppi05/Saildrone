import numpy as np
import matplotlib.pyplot as plt

from ode import solve_ivp
from programme import dynamics
#simulate
def simulate():
    t0 = 0
    tmax = 120
    dt = 0.05
#initial state
    z0 = np.array([
        0.0, #initial x position
        0.0, #initial y position
        np.pi/2, #initial heading angle
        0.0, #initial x velocity
        2.9, #initial y velocity
        0.0  #initial angular velocity
    ])
#solve ODE
    t, z = solve_ivp(dynamics, t0, tmax, dt, z0, method='RK')
    return t, z

#plots trajectory of sail drone
def plot_trajectory(t, z):
    x = z[0] 
    y = z[1] 

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, 'b', linewidth=2)

    times = [0, 60, 65]
# mark specific times
    for time in times:
        i = (np.abs(t - time)).argmin()
        plt.scatter(x[i], y[i], c='red', s=100, marker='o', zorder=10)
        plt.text(x[i], y[i], f" t={time}s", fontsize=9)

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
#main function to run simulation and plot
if __name__ == "__main__":
    t, z = simulate()
    plot_trajectory(t, z)
