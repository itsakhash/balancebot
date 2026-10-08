"""Step 20: estimate tilt from a noisy accelerometer and a drifting gyro (complementary filter).

The robot motion comes from the simulated balancing robot. The sensors are faked with the
faults real ones have, so we can compare each estimate against the true tilt.
"""
import math
import random

import matplotlib
matplotlib.use("Agg")  # draw to a file, no window needed
import matplotlib.pyplot as plt

G = 9.81
L = 0.10
DT = 0.005          # 200 Hz
T_END = 10.0
MAX_ACCEL = 5.0
KP, KD = 30.0, 1.5  # the baseline controller from step 6 (uses the TRUE angle here)

# Assumed sensor faults (made up for the simulation; real values come from the hardware).
GYRO_BIAS_DEG_S = 1.5     # the gyro reads this much even when the robot is still
GYRO_DRIFT_DEG_S2 = 0.1   # and that offset creeps up this much every second (warm-up drift)
GYRO_NOISE_DEG_S = 0.5    # random noise on each gyro reading (std dev)
ACCEL_NOISE_G = 0.01      # random noise on each accelerometer axis (std dev)

SHOVES = {1.5: 40.0, 4.0: -30.0, 6.5: 50.0, 9.0: -20.0}  # time (s) -> sudden spin (deg/s)


def make_recording(seed=1):
    """Run the balancing robot and record the truth plus fake sensor readings."""
    rng = random.Random(seed)
    theta = 0.0                 # starts upright and still, like a robot held steady at power-up
    omega = 0.0
    steps = int(T_END / DT)
    shove_steps = {round(t / DT): math.radians(d) for t, d in SHOVES.items()}
    rec = {"t": [], "theta": [], "gyro": [], "acc_angle": []}
    for k in range(steps):
        if k in shove_steps:
            omega += shove_steps[k]

        accel = -(KP * (-theta) + KD * (-omega))
        accel = max(-MAX_ACCEL, min(MAX_ACCEL, accel))

        # What an accelerometer at the axle would feel (gravity plus the wheels' push),
        # expressed along the robot's own forward and up axes.
        a_forward = accel * math.cos(theta) - G * math.sin(theta)
        a_up = accel * math.sin(theta) + G * math.cos(theta)
        a_forward += rng.gauss(0.0, ACCEL_NOISE_G) * G
        a_up += rng.gauss(0.0, ACCEL_NOISE_G) * G

        rec["t"].append(k * DT)
        rec["theta"].append(math.degrees(theta))
        bias = GYRO_BIAS_DEG_S + GYRO_DRIFT_DEG_S2 * (k * DT)
        rec["gyro"].append(math.degrees(omega) + bias + rng.gauss(0.0, GYRO_NOISE_DEG_S))
        rec["acc_angle"].append(math.degrees(math.atan2(-a_forward, a_up)))

        alpha_ang = (G * math.sin(theta) - accel * math.cos(theta)) / L
        omega += alpha_ang * DT
        theta += omega * DT
    return rec


CAL_SECONDS = 0.5   # the robot is still for this long at the start; average the gyro then


def calibrate(rec):
    """Return a copy of the recording with the gyro's resting offset subtracted."""
    n = int(CAL_SECONDS / DT)
    offset = sum(rec["gyro"][:n]) / n
    cal = dict(rec)
    cal["gyro"] = [w - offset for w in rec["gyro"]]
    return cal, offset


def gyro_only(rec):
    est, out = rec["theta"][0], []
    for w in rec["gyro"]:
        est += w * DT
        out.append(est)
    return out


def complementary(rec, alpha):
    """est = alpha * (est + gyro * dt) + (1 - alpha) * accelerometer_angle"""
    est, out = rec["acc_angle"][0], []
    for w, a in zip(rec["gyro"], rec["acc_angle"]):
        est = alpha * (est + w * DT) + (1.0 - alpha) * a
        out.append(est)
    return out


def errors(est, truth):
    err = [e - t for e, t in zip(est, truth)]
    rms = math.sqrt(sum(x * x for x in err) / len(err))
    return rms, max(abs(x) for x in err), err


rec = make_recording()
truth = rec["theta"]
cal, offset = calibrate(rec)
print(f"gyro resting offset measured during calibration: {offset:.2f} deg/s")
print()

candidates = {
    "accelerometer only": rec["acc_angle"],
    "gyro only, uncalibrated": gyro_only(rec),
    "gyro only, calibrated": gyro_only(cal),
    "compl. a=0.98, uncalibrated": complementary(rec, 0.98),
    "compl. a=0.90, calibrated": complementary(cal, 0.90),
    "compl. a=0.98, calibrated": complementary(cal, 0.98),
    "compl. a=0.995, calibrated": complementary(cal, 0.995),
}

print(f"{'method':28s} {'rms error':>10s} {'max error':>10s}")
errs = {}
for name, est in candidates.items():
    rms, worst, err = errors(est, truth)
    errs[name] = err
    print(f"{name:28s} {rms:8.2f} deg {worst:7.2f} deg")

# time constant of the chosen filter: tau = alpha * dt / (1 - alpha)
for a in (0.90, 0.98, 0.995):
    print(f"alpha {a}: time constant = {a * DT / (1 - a) * 1000:.0f} ms")

fig, ax = plt.subplots(2, 1, sharex=True, figsize=(8, 7))
ax[0].plot(rec["t"], truth, "k", label="true tilt", linewidth=2)
ax[0].plot(rec["t"], candidates["accelerometer only"], label="accelerometer only", alpha=0.5)
ax[0].plot(rec["t"], candidates["gyro only, uncalibrated"], label="gyro only, uncalibrated", alpha=0.8)
ax[0].plot(rec["t"], candidates["compl. a=0.98, calibrated"], label="complementary a=0.98, calibrated")
ax[0].set_ylabel("tilt (deg)")
ax[0].legend(fontsize=8)
ax[0].grid(True)
for name in ("accelerometer only", "gyro only, uncalibrated", "compl. a=0.98, calibrated"):
    ax[1].plot(rec["t"], errs[name], label=name, alpha=0.8)
ax[1].set_ylabel("error vs truth (deg)")
ax[1].set_xlabel("time (s)")
ax[1].legend(fontsize=8)
ax[1].grid(True)
fig.suptitle("Tilt estimation from simulated IMU data")
fig.savefig("complementary_filter.png", dpi=120)
print("Saved complementary_filter.png")
