"""Recompute the balance gains for the real chassis and test them with your measured sensor noise.

Run it again after you weigh and balance the finished robot:

    python3 design_gains.py --L 0.062

It prints
  1. the LQR gains for that centre-of-mass height, as a block to paste into controller.h
  2. a survival table: how often PID, the LQR gains for this L, and the OLD gains (L = 0.10)
     keep the robot upright over 20 noisy runs, with the sensor model below
  3. with --margins, how much sensing-to-wheel delay, motor lag and how little wheel acceleration it survives
  4. with --reference, the ideal-sensor checkpoints used by firmware/test/test_controller.c

Sensor model (the same chain the firmware runs): accelerometer noise and gyro noise go through the
200 Hz complementary filter (alpha 0.98); the wheel position is quantised to one encoder count and
the speed is a low-pass filtered difference (0.2 per step), exactly like balance.c.
Only numpy is needed.
"""
import argparse
import math
from collections import deque

import numpy as np

G = 9.81
DT = 0.005                 # 200 Hz
MAX_ACCEL = 5.0            # m/s^2 the wheels can add (placeholder until measured)
CUT_RAD = math.radians(35.0)   # firmware cut-off
ALPHA = 0.98               # complementary filter weight on the gyro
SPEED_LP = 0.2             # balance.c BALANCE_SPEED_LP
COUNT_M = math.pi * 0.065 / (11 * 4 * 21.3)   # metres of wheel travel per encoder count
Q = np.diag([10.0, 1.0, 100.0, 1.0])
R = np.array([[1.0]])
OLD_L = 0.10
OLD_K = np.array([-3.16227766, -3.71901666, -29.71225976, -2.99663144])


def model(L):
    """x'' = u, theta'' = (g/L) theta - u/L with state [position, speed, tilt, tilt rate]."""
    A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, G / L, 0]], dtype=float)
    B = np.array([[0], [1], [0], [-1 / L]], dtype=float)
    return A, B


def lqr_gains(L, q=Q, r=R):
    """Continuous LQR by the Hamiltonian method. Returns K with u = -K z."""
    A, B = model(L)
    n = A.shape[0]
    r_inv = np.linalg.inv(r)
    ham = np.block([[A, -B @ r_inv @ B.T], [-q, -A.T]])
    values, vectors = np.linalg.eig(ham)
    stable = vectors[:, values.real < 0]
    x_part, y_part = stable[:n, :], stable[n:, :]
    p = np.real(y_part @ np.linalg.inv(x_part))
    return (r_inv @ B.T @ p).ravel()


def pid_accel(est, rate):
    return 30.0 * est + 1.5 * rate


def simulate(L, controller, noisy=True, delay_steps=3, motor_lag=0.03, accel_noise_g=0.003,
             gyro_noise_dps=0.35, t_end=8.0, seed=1, max_accel=MAX_ACCEL):
    """3 degree lean at the start, 40 deg/s shove at 2 s. controller(x, v, est, rate) -> accel.
    noisy=False feeds the controller the true state (like lqr_balance.py); noisy=True runs the
    sensor chain described at the top. Returns (record, step it passed the 35 degree cut-off or None)."""
    rng = np.random.default_rng(seed)
    x = v = om = 0.0
    th = math.radians(3.0)
    est = th                       # filter seeded from the accelerometer, as in tilt.c
    resid = math.radians(rng.normal(0, gyro_noise_dps / math.sqrt(200))) if noisy else 0.0
    x_meas_prev = 0.0
    v_est = 0.0
    accel_actual = 0.0
    in_flight = deque([0.0] * delay_steps)
    rec = {"t": [], "x": [], "v": [], "tilt": []}
    for k in range(int(t_end / DT)):
        t = k * DT
        if abs(t - 2.0) < DT / 2:
            om += math.radians(40.0)

        if noisy:
            accel_angle = th + rng.normal(0, accel_noise_g)           # atan2(-ax, az) ~ theta + noise / 1 g
            rate = om + math.radians(rng.normal(0, gyro_noise_dps)) + resid
            x_meas = round(x / COUNT_M) * COUNT_M
            est = ALPHA * (est + rate * DT) + (1 - ALPHA) * accel_angle
            v_est += SPEED_LP * ((x_meas - x_meas_prev) / DT - v_est)
            x_meas_prev = x_meas
        else:
            est, rate, x_meas, v_est = th, om, x, v

        cmd = max(-max_accel, min(max_accel, controller(x_meas, v_est, est, rate)))
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
        rec["t"].append(t); rec["x"].append(x); rec["v"].append(v); rec["tilt"].append(math.degrees(th))
        if abs(th) > CUT_RAD:
            return rec, k
    return rec, None


def lqr_controller(K):
    return lambda x, v, est, rate: float(-(K[0] * x + K[1] * v + K[2] * est + K[3] * rate))


def pid_controller():
    return lambda x, v, est, rate: pid_accel(est, rate)


def survival(L, controller, runs=20, **kw):
    """Run the noisy simulation with seeds 1..runs. Returns (stayed up, mean |final speed|, mean |final position|, mean max tilt after the shove)."""
    ups, speeds, positions, worst = 0, [], [], []
    for seed in range(1, runs + 1):
        rec, fell = simulate(L, controller, seed=seed, **kw)
        if fell is None:
            ups += 1
            speeds.append(abs(rec["v"][-1]))
            positions.append(abs(rec["x"][-1]))
            worst.append(max(abs(a) for a, t in zip(rec["tilt"], rec["t"]) if t > 2.0))
    mean = lambda a: sum(a) / len(a) if a else float("nan")
    return ups, mean(speeds), mean(positions), mean(worst)


def header_block(K):
    names = ["position m", "speed m/s", "tilt rad", "tilt rate rad/s"]
    lines = ["/* u = -K z with z = [position m, speed m/s, tilt rad, tilt rate rad/s] */"]
    for i, k in enumerate(K):
        lines.append(f"#define BALANCE_LQR_K{i} ({k:.8f}f)")
    return "\n".join(lines)


def reference_points(L, K):
    """Ideal-sensor checkpoints (no noise, no delay, no lag) for firmware/test/test_controller.c."""
    steps = [200, 400, 600, 800, 1200, 1598]
    out = {}
    for name, ctrl in (("PID", pid_controller()), ("LQR", lqr_controller(K))):
        rec, fell = simulate(L, ctrl, noisy=False, delay_steps=0, motor_lag=0.0)
        # the ideal check in the C test feeds the true state, so use the true tilt, not the filter
        out[name] = (fell, [(rec["t"][i], rec["tilt"][i], rec["v"][i], rec["x"][i]) for i in steps if i < len(rec["t"])])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--L", type=float, default=0.062, help="centre of mass height above the axle, metres")
    ap.add_argument("--accel-noise-g", type=float, default=0.003)
    ap.add_argument("--gyro-noise-dps", type=float, default=0.35)
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--sweep", action="store_true", help="also compare several L values")
    ap.add_argument("--margins", action="store_true", help="latency budget and wheel-acceleration limit the controllers survive")
    ap.add_argument("--reference", action="store_true", help="print the ideal checkpoints for test_controller.c")
    args = ap.parse_args()

    kw = dict(accel_noise_g=args.accel_noise_g, gyro_noise_dps=args.gyro_noise_dps)
    K = lqr_gains(args.L)
    A, B = model(args.L)
    poles = np.linalg.eigvals(A - B @ K.reshape(1, 4))
    print(f"L = {args.L:.3f} m")
    print("closed-loop poles:", np.round(np.sort_complex(poles), 2))
    print()
    print("Paste into firmware/include/controller.h:")
    print(header_block(K))
    print()

    def row(label, L, ctrl):
        ups, v, x, w = survival(L, ctrl, runs=args.runs, **kw)
        print(f"{label:34s} {ups:3d}/{args.runs}  {w:10.2f} deg {v:11.3f} m/s {x:11.3f} m")

    print(f"Noisy runs ({args.accel_noise_g} g accel, {args.gyro_noise_dps} deg/s gyro, 15 ms delay, 30 ms motor lag):")
    print(f"{'controller':34s} {'stayed up':>9s}  {'mean max tilt':>13s} {'|final speed|':>13s} {'|final pos|':>13s}")
    row("PID", args.L, pid_controller())
    row("LQR, gains for this L", args.L, lqr_controller(K))
    row("LQR, old gains (L = 0.10)", args.L, lqr_controller(OLD_K))

    if args.sweep:
        print()
        print("Same, over a range of centre-of-mass heights:")
        print(f"{'L (m)':>8s} {'LQR matched':>12s} {'LQR old K':>10s} {'PID':>6s}")
        for L in (0.05, 0.06, 0.08, 0.10, 0.12, 0.15):
            matched = survival(L, lqr_controller(lqr_gains(L)), runs=args.runs, **kw)[0]
            stale = survival(L, lqr_controller(OLD_K), runs=args.runs, **kw)[0]
            pid = survival(L, pid_controller(), runs=args.runs, **kw)[0]
            print(f"{L:8.2f} {matched:9d}/{args.runs} {stale:7d}/{args.runs} {pid:3d}/{args.runs}")

    if args.margins:
        n = min(args.runs, 10)
        print()
        print(f"Latency budget: stayed up out of {n} noisy runs, LQR / PID (delay = sensing to wheel command, lag = motor response):")
        delays = (0, 10, 15, 20, 25, 30)
        print(f"{'motor lag':>10s} " + " ".join(f"{d:>5d} ms" for d in delays))
        for lag in (0.01, 0.03, 0.05):
            cells = []
            for d in delays:
                steps = round(d / (DT * 1000))
                lq = survival(args.L, lqr_controller(K), runs=n, delay_steps=steps, motor_lag=lag, **kw)[0]
                pd = survival(args.L, pid_controller(), runs=n, delay_steps=steps, motor_lag=lag, **kw)[0]
                cells.append(f"{lq:>2d}/{pd:<2d}  ")
            print(f"{lag * 1000:7.0f} ms  " + " ".join(cells))
        print()
        print(f"Wheel acceleration limit (LQR, 15 ms delay, 30 ms lag), stayed up out of {n}:")
        for a_max in (5.0, 3.0, 2.0, 1.5, 1.0, 0.75):
            ups = survival(args.L, lqr_controller(K), runs=n, max_accel=a_max, **kw)[0]
            print(f"  {a_max:4.2f} m/s^2: {ups}/{n}")

    if args.reference:
        print()
        print("Ideal-sensor checkpoints for firmware/test/test_controller.c (t, tilt deg, speed m/s, position m):")
        for name, (fell, pts) in reference_points(args.L, K).items():
            print(f"{name}: fell at step {fell}")
            for t, tilt, v, x in pts:
                print(f"  t={t:.3f} tilt={tilt:.6f} v={v:.6f} x={x:.6f}")


if __name__ == "__main__":
    main()