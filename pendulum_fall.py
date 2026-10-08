"""Step 2: an inverted pendulum with NO controller. It should fall over."""
import math

import matplotlib
matplotlib.use("Agg")  # draw to a file, no window needed
import matplotlib.pyplot as plt

G = 9.81      # gravity, m/s^2
L = 0.10      # height of the robot's center of mass above the axle, m
DT = 0.005    # time step, s (200 Hz, the loop rate we will use on the Nucleo)
T_END = 1.5   # how long to simulate, s

theta = math.radians(3.0)   # start leaning 3 degrees forward
omega = 0.0                 # angular velocity, rad/s
accel = 0.0                 # wheel acceleration (the control input), m/s^2: zero for now

times, angles = [], []
fell_at = None

t = 0.0
while t < T_END:
    times.append(t)
    angles.append(math.degrees(theta))

    # Pendulum dynamics: gravity pulls it over, wheel acceleration can push it back.
    alpha = (G * math.sin(theta) - accel * math.cos(theta)) / L
    omega += alpha * DT
    theta += omega * DT
    t += DT

    if fell_at is None and abs(theta) > math.pi / 2:
        fell_at = t
        break

if fell_at is not None:
    print(f"Fell over (more than 90 degrees) at t = {fell_at:.2f} s")
else:
    print("Did not fall over")

plt.plot(times, angles)
plt.xlabel("time (s)")
plt.ylabel("tilt angle (degrees)")
plt.title("Inverted pendulum, no controller")
plt.grid(True)
plt.savefig("pendulum_fall.png", dpi=120)
print("Saved pendulum_fall.png")
