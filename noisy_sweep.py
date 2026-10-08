"""Step 8: the same balance test, but with sensor noise, a control delay, and motor lag."""
import math
import random
from collections import deque

import matplotlib
matplotlib.use("Agg")  # draw to a file, no window needed
import matplotlib.pyplot as plt

G = 9.81
L = 0.10
DT = 0.005
T_END = 4.0
MAX_ACCEL = 5.0

ANGLE_NOISE_DEG = 0.3      # random error on the tilt estimate (std dev)
GYRO_NOISE_DEG_S = 3.0     # random error on the angular-rate reading (std dev)
DELAY_STEPS = 3            # command takes 3 * 5 ms = 15 ms to reach the wheels
MOTOR_LAG_S = 0.03         # motors take about 30 ms to follow a command


def simulate(kp, ki, kd, seed=1):
    rng = random.Random(seed)
    theta = math.radians(3.0)
    omega = 0.0
    integral = 0.0
    accel_actual = 0.0
    pending = deque([0.0] * DELAY_STEPS)   # commands still "in flight"
    times, angles, commands = [], [], []
    t = 0.0
    while t < T_END:
        if abs(t - 2.0) < DT / 2:
            omega += math.radians(40.0)

        # What the controller sees: the truth plus noise.
        theta_meas = theta + math.radians(rng.gauss(0.0, ANGLE_NOISE_DEG))
        omega_meas = omega + math.radians(rng.gauss(0.0, GYRO_NOISE_DEG_S))

        integral += (-theta_meas) * DT
        cmd = -(kp * (-theta_meas) + ki * integral + kd * (-omega_meas))
        cmd = max(-MAX_ACCEL, min(MAX_ACCEL, cmd))

        # Delay, then motor lag.
        pending.append(cmd)
        delayed = pending.popleft()
        accel_actual += (delayed - accel_actual) * (DT / MOTOR_LAG_S)

        alpha = (G * math.sin(theta) - accel_actual * math.cos(theta)) / L
        omega += alpha * DT
        theta += omega * DT

        times.append(t)
        angles.append(math.degrees(theta))
        commands.append(cmd)
        t += DT

        if abs(theta) > math.pi / 2:
            return times, angles, commands, t
    return times, angles, commands, None


CASES = [
    ("baseline",           30.0, 0.0, 1.5),
    ("kp very high (200)", 200.0, 0.0, 4.0),
    ("kd too high (8)",    30.0, 0.0, 8.0),
    ("kp 60, kd 2.5",      60.0, 0.0, 2.5),
]

fig, axes = plt.subplots(len(CASES), 1, sharex=True, figsize=(7, 8))
print(f"{'case':22s} {'result':16s} {'tilt jitter (rms)':>18s} {'command jitter (rms)':>22s}")
for ax, (name, kp, ki, kd) in zip(axes, CASES):
    times, angles, commands, fell = simulate(kp, ki, kd)
    if fell is not None:
        result = f"fell at {fell:.2f} s"
        tilt_rms = cmd_rms = "-"
    else:
        result = "stayed up"
        # jitter measured in the quiet period 1.0 s to 1.9 s, before the shove
        quiet = [(a, c) for a, c, tt in zip(angles, commands, times) if 1.0 <= tt < 1.9]
        tilt_rms = f"{math.sqrt(sum(a * a for a, _ in quiet) / len(quiet)):.2f} deg"
        cmd_rms = f"{math.sqrt(sum(c * c for _, c in quiet) / len(quiet)):.2f} m/s^2"
    print(f"{name:22s} {result:16s} {tilt_rms:>18s} {cmd_rms:>22s}")
    ax.plot(times, angles)
    ax.set_ylabel("tilt (deg)")
    ax.set_title(f"{name}: kp={kp}, kd={kd}", fontsize=9)
    ax.set_ylim(-10, 10)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
fig.tight_layout()
fig.savefig("noisy_sweep.png", dpi=110)
print("Saved noisy_sweep.png")
