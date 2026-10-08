"""Step 22: fix the drift with full state feedback (LQR) and compare it with the PID loop.

The PID controller only sees tilt, so the robot stays upright but rolls away. State
feedback also sees position and speed. Only numpy is needed (no scipy).
"""
import math
from collections import deque

import matplotlib
matplotlib.use("Agg")  # draw to a file, no window needed
import matplotlib.pyplot as plt
import numpy as np

G = 9.81
L = 0.10
DT = 0.005
MAX_ACCEL = 5.0

# Linearized model about upright, state z = [position, speed, tilt, tilt rate],
# input u = wheel acceleration:   x'' = u,   theta'' = (g/L) * theta - u / L
A = np.array([[0, 1, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 1],
              [0, 0, G / L, 0]], dtype=float)
B = np.array([[0], [1], [0], [-1 / L]], dtype=float)


def lqr(A, B, Q, R):
    """Solve the continuous LQR problem with the Hamiltonian-matrix method.

    Returns K such that u = -K z minimizes the integral of z'Qz + u'Ru.
    """
    n = A.shape[0]
    r_inv = np.linalg.inv(R)
    hamiltonian = np.block([[A, -B @ r_inv @ B.T],
                            [-Q, -A.T]])
    values, vectors = np.linalg.eig(hamiltonian)
    stable = vectors[:, values.real < 0]          # the n eigenvectors with negative real part
    x_part, y_part = stable[:n, :], stable[n:, :]
    p = np.real(y_part @ np.linalg.inv(x_part))
    return (r_inv @ B.T @ p).ravel()


Q = np.diag([10.0, 1.0, 100.0, 1.0])   # how much we care about position, speed, tilt, tilt rate
R = np.array([[1.0]])                  # how much we care about using wheel acceleration
K = lqr(A, B, Q, R)


def pid_controller(z):
    """The step 6 controller: it only looks at tilt and tilt rate."""
    return 30.0 * z[2] + 1.5 * z[3]


def lqr_controller(z):
    return float(-(K @ z))


def simulate(controller, t_end=8.0, delay_steps=0, motor_lag=0.0, noisy=False, seed=1):
    """3 degree lean at the start, a shove at 2 s. Returns (time, position, speed, tilt deg), fell_at."""
    rng = np.random.default_rng(seed)
    x = v = om = 0.0
    th = math.radians(3.0)
    accel_actual = 0.0
    in_flight = deque([0.0] * delay_steps)
    out = {"t": [], "x": [], "v": [], "tilt": []}
    for k in range(int(t_end / DT)):
        t = k * DT
        if abs(t - 2.0) < DT / 2:
            om += math.radians(40.0)

        z = np.array([x, v, th, om])
        if noisy:   # same sensor noise as step 8 on tilt and tilt rate
            z = z + np.array([0.0, 0.0,
                              math.radians(rng.normal(0, 0.3)),
                              math.radians(rng.normal(0, 3.0))])
        cmd = max(-MAX_ACCEL, min(MAX_ACCEL, controller(z)))

        if delay_steps:
            in_flight.append(cmd)
            cmd = in_flight.popleft()
        if motor_lag > 0:
            accel_actual += (cmd - accel_actual) * (DT / motor_lag)
        else:
            accel_actual = cmd

        om += (G * math.sin(th) - accel_actual * math.cos(th)) / L * DT
        th += om * DT
        v += accel_actual * DT
        x += v * DT

        out["t"].append(t)
        out["x"].append(x)
        out["v"].append(v)
        out["tilt"].append(math.degrees(th))
        if abs(th) > math.pi / 2:
            return out, t
    return out, None


print("open-loop poles :", np.round(np.sort_complex(np.linalg.eigvals(A)), 2))
print("LQR gains K     :", np.round(K, 2), " (u = -K z)")
print("closed-loop poles:", np.round(np.sort_complex(np.linalg.eigvals(A - B @ K.reshape(1, 4))), 2))
print()

# Test 1: ideal sensors and motors
print("Ideal sensors and motors, 8 s run:")
print(f"{'controller':10s} {'result':10s} {'max tilt after shove':>21s} {'final speed':>12s} {'final position':>15s}")
runs = {}
for name, ctrl in (("PID", pid_controller), ("LQR", lqr_controller)):
    res, fell = simulate(ctrl)
    runs[name] = res
    worst = max(abs(a) for a, t in zip(res["tilt"], res["t"]) if t > 2.0)
    print(f"{name:10s} {'stayed up' if fell is None else f'fell {fell:.2f}s':10s} "
          f"{worst:18.2f} deg {res['v'][-1]:8.3f} m/s {res['x'][-1]:11.3f} m")
print()

# Test 2: noise, 15 ms delay and 30 ms motor lag (the step 8 conditions), 20 noise seeds
print("With noise, 15 ms delay and 30 ms motor lag, 20 random seeds:")
print(f"{'controller':10s} {'stayed up':>10s} {'mean |final speed|':>20s} {'mean |final position|':>23s}")
for name, ctrl in (("PID", pid_controller), ("LQR", lqr_controller)):
    ups, speeds, positions = 0, [], []
    for seed in range(1, 21):
        res, fell = simulate(ctrl, delay_steps=3, motor_lag=0.03, noisy=True, seed=seed)
        if fell is None:
            ups += 1
            speeds.append(abs(res["v"][-1]))
            positions.append(abs(res["x"][-1]))
    mean_v = sum(speeds) / len(speeds) if speeds else float("nan")
    mean_x = sum(positions) / len(positions) if positions else float("nan")
    print(f"{name:10s} {ups:7d}/20 {mean_v:16.3f} m/s {mean_x:19.3f} m")

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
for name, color in (("PID", "tab:orange"), ("LQR", "tab:blue")):
    ax[0].plot(runs[name]["t"], runs[name]["tilt"], color=color, label=name)
    ax[1].plot(runs[name]["t"], runs[name]["v"], color=color, label=name)
    ax[2].plot(runs[name]["t"], runs[name]["x"], color=color, label=name)
ax[0].set_ylabel("tilt (deg)")
ax[1].set_ylabel("speed (m/s)")
ax[2].set_ylabel("position (m)")
ax[2].set_xlabel("time (s)")
for a in ax:
    a.grid(True)
    a.legend()
fig.suptitle("PID vs LQR: same start, same shove at t = 2 s")
fig.savefig("lqr_balance.png", dpi=120)
print("Saved lqr_balance.png")
