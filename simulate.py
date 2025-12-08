
#code below does not plot point at 60s

import numpy as np
import matplotlib.pyplot as plt

from ode import solve_ivp
from programme import dynamics

def simulate():

    t0 = 0
    tmax = 120
    dt = 0.05

    z0 = np.array([
        0.0,
        0.0,
        np.pi/2,
        0.0,
        2.9,
        0.0         
    ])

    t, z = solve_ivp(dynamics, t0, tmax, dt, z0, method='RK')
    return t, z


def plot_trajectory(t, z):
    x = z[0]
    y = z[1] 

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, 'b', linewidth=2)


    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()



if __name__ == "__main__":
    t, z = simulate()
    plot_trajectory(t, z)

'''
#DONT USE CODE BELOW FOR FINAL REPORT (CHATGPT)
import numpy as np
import matplotlib.pyplot as plt

from ode import solve_ivp
from programme import dynamics

def simulate():
    t0 = 0
    tmax = 120
    dt = 0.05

    z0 = np.array([
        0.0, 
        0.0, 
        np.pi/2,
        0.0,
        2.9,  
        0.0  
    ])

    t, z = solve_ivp(dynamics, t0, tmax, dt, z0, method='RK')
    return t, z


def plot_trajectory_with_marker(t, z, t_marker=60.0):
    x = z[0]
    y = z[1]

    idx = np.argmin(np.abs(t - t_marker))

    x_m = x[idx]
    y_m = y[idx]

    print(f"Closest time to {t_marker}s: {t[idx]:.3f} s")
    print(f"x = {x_m:.3f},  y = {y_m:.3f}")
    print(f"theta = {z[2,idx]:.3f} rad")
    print(f"vx = {z[3,idx]:.3f},  vy = {z[4,idx]:.3f}")
    print(f"omega = {z[5,idx]:.5f}")

   
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, 'b', linewidth=2, label="Trajectory")


    plt.scatter(x_m, y_m, color='red', s=80, label=f"t = {t_marker}s")
    plt.text(x_m, y_m, f"   {t_marker}s", fontsize=10, color='red')

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Saildrone Trajectory with 60-second Marker")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()
'''

if __name__ == "__main__":
    t, z = simulate()
    plot_trajectory(t, z)
