"""Step 6: balance the simulated inverted pendulum with a PID controller."""
import math

import matplotlib
matplotlib.use("Agg")  # draw to a file, no window needed
import matplotlib.pyplot as plt

G = 9.81        # gravity, m/s^2
L = 0.10        # height of the center of mass above the axle, m
DT = 0.005      # time step, s (200 Hz)
T_END = 4.0     # simulated time, s
MAX_ACCEL = 5.0 # the most the wheels can accelerate the robot, m/s^2


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0

    def update(self, error, error_rate, dt):
        self.integral += error * dt
        return self.kp * error + self.ki * self.integral + self.kd * error_rate


# Gains: the robot must accelerate toward the lean, so kp must beat gravity (kp > 9.81).
pid = PID(kp=30.0, ki=0.0, kd=1.5)

theta = math.radians(3.0)   # start leaning 3 degrees forward
omega = 0.0                 # angular velocity, rad/s
speed = 0.0                 # robot speed along the floor, m/s

times, angles, accels, speeds = [], [], [], []

t = 0.0
while t < T_END:
    # A shove at t = 2 s: a sudden spin of the robot, like a push on the top.
    if abs(t - 2.0) < DT / 2:
        omega += math.radians(40.0)

    # Controller: setpoint is upright (0 rad), so error = -theta. Lean forward -> drive forward.
    accel = -pid.update(-theta, -omega, DT)
    accel = max(-MAX_ACCEL, min(MAX_ACCEL, accel))

    # Pendulum dynamics.
    alpha = (G * math.sin(theta) - accel * math.cos(theta)) / L
    omega += alpha * DT
    theta += omega * DT
    speed += accel * DT

    times.append(t)
    angles.append(math.degrees(theta))
    accels.append(accel)
    speeds.append(speed)
    t += DT

    if abs(theta) > math.pi / 2:
        print(f"Fell over at t = {t:.2f} s")
        break
else:
    print(f"Stayed upright for {T_END:.0f} s")

print(f"Max tilt after the shove: {max(abs(a) for a, tt in zip(angles, times) if tt > 2.0):.1f} deg")
print(f"Final tilt: {angles[-1]:.3f} deg")
print(f"Final speed: {speeds[-1]:.2f} m/s")

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(7, 7))
ax[0].plot(times, angles); ax[0].set_ylabel("tilt (deg)")
ax[1].plot(times, accels); ax[1].set_ylabel("wheel accel (m/s^2)")
ax[2].plot(times, speeds); ax[2].set_ylabel("speed (m/s)"); ax[2].set_xlabel("time (s)")
for a in ax:
    a.grid(True)
fig.suptitle("PID balance, shove at t = 2 s")
fig.savefig("pid_balance.png", dpi=120)
print("Saved pid_balance.png")
