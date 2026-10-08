"""Step 7: change one PID gain at a time and watch how the balance changes."""
import math

import matplotlib
matplotlib.use("Agg")  # draw to a file, no window needed
import matplotlib.pyplot as plt

G = 9.81
L = 0.10
DT = 0.005
T_END = 4.0
MAX_ACCEL = 5.0


def simulate(kp, ki, kd):
    """Run one balance test. Returns (times, tilt angles in degrees, time it fell or None)."""
    theta = math.radians(3.0)
    omega = 0.0
    integral = 0.0
    times, angles = [], []
    t = 0.0
    while t < T_END:
        if abs(t - 2.0) < DT / 2:          # the same shove at t = 2 s
            omega += math.radians(40.0)

        integral += (-theta) * DT
        accel = -(kp * (-theta) + ki * integral + kd * (-omega))
        accel = max(-MAX_ACCEL, min(MAX_ACCEL, accel))

        alpha = (G * math.sin(theta) - accel * math.cos(theta)) / L
        omega += alpha * DT
        theta += omega * DT

        times.append(t)
        angles.append(math.degrees(theta))
        t += DT

        if abs(theta) > math.pi / 2:
            return times, angles, t
    return times, angles, None


# name, kp, ki, kd  (only one thing differs from the baseline in each row)
CASES = [
    ("baseline",         30.0, 0.0, 1.5),
    ("kp too low (8)",    8.0, 0.0, 1.5),
    ("no damping (kd=0)", 30.0, 0.0, 0.0),
    ("kd too high (8)",  30.0, 0.0, 8.0),
    ("kp very high (200)", 200.0, 0.0, 4.0),
    ("with ki (40)",     30.0, 40.0, 1.5),
]

fig, axes = plt.subplots(len(CASES), 1, sharex=True, figsize=(7, 10))
print(f"{'case':22s} {'result':16s} {'max tilt after shove':>22s}")
for ax, (name, kp, ki, kd) in zip(axes, CASES):
    times, angles, fell = simulate(kp, ki, kd)
    if fell is not None:
        result = f"fell at {fell:.2f} s"
        worst = "-"
    else:
        result = "stayed up"
        worst = f"{max(abs(a) for a, tt in zip(angles, times) if tt > 2.0):.1f} deg"
    print(f"{name:22s} {result:16s} {worst:>22s}")
    ax.plot(times, angles)
    ax.set_ylabel("tilt (deg)")
    ax.set_title(f"{name}: kp={kp}, ki={ki}, kd={kd}", fontsize=9)
    ax.set_ylim(-10, 10)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
fig.tight_layout()
fig.savefig("gain_sweep.png", dpi=110)
print("Saved gain_sweep.png")
