"""Tests for design_gains.py. Run:  python3 test_design_gains.py"""
import math

import numpy as np

import design_gains as dg

failures = 0


def check(cond, msg):
    global failures
    if cond:
        print("ok:  ", msg)
    else:
        print("FAIL:", msg)
        failures += 1


# 1. Same answer as the gains already in the firmware (L = 0.10).
K = dg.lqr_gains(0.10)
check(np.allclose(K, dg.OLD_K, atol=1e-6), "LQR gains at L = 0.10 match controller.h")

# 2. Closed loop is stable for several heights.
for L in (0.05, 0.062, 0.10, 0.15):
    A, B = dg.model(L)
    Kl = dg.lqr_gains(L)
    poles = np.linalg.eigvals(A - B @ Kl.reshape(1, 4))
    check(all(p.real < 0 for p in poles), f"closed-loop poles stable at L = {L}")

# 3. The gains hardly depend on L (a finding of this script): tilt gain moves less than 10 percent over 0.05..0.15 m.
k_short, k_tall = dg.lqr_gains(0.05)[2], dg.lqr_gains(0.15)[2]
check(abs(k_short - k_tall) / abs(k_tall) < 0.10, "tilt gain changes by less than 10 percent between L = 0.05 and 0.15")

# 4. Signs: k2 and k3 negative, so u = -K z drives toward a lean (same convention as the firmware).
check(K[2] < 0 and K[3] < 0, "tilt and tilt-rate gains are negative, as in the firmware")

# 5. Ideal simulation: both controllers stay upright at L = 0.10 and LQR settles.
for name, ctrl in (("PID", dg.pid_controller()), ("LQR", dg.lqr_controller(K))):
    rec, fell = dg.simulate(0.10, ctrl, noisy=False, delay_steps=0, motor_lag=0.0)
    check(fell is None, f"{name} stays upright in the ideal simulation")
rec, _ = dg.simulate(0.10, dg.lqr_controller(K), noisy=False, delay_steps=0, motor_lag=0.0)
check(abs(rec["v"][-1]) < 0.02, "LQR speed settles near zero (ideal)")

# 5b. Ideal-state checkpoints at L = 0.10 equal the numbers printed by lqr_balance.py / used in test_controller.c.
rec, _ = dg.simulate(0.10, dg.pid_controller(), noisy=False, delay_steps=0, motor_lag=0.0)
check(abs(rec["tilt"][400] - 0.185000) < 1e-3 and abs(rec["v"][600] - 0.139444) < 1e-3, "ideal PID checkpoints match the Python reference")
rec, _ = dg.simulate(0.10, dg.lqr_controller(K), noisy=False, delay_steps=0, motor_lag=0.0)
check(abs(rec["tilt"][200] - (-0.271499)) < 1e-3 and abs(rec["x"][200] - 0.031289) < 1e-3, "ideal LQR checkpoints match the Python reference")

# 6. Cut-off is reported when the robot falls (zero controller).
rec, fell = dg.simulate(0.10, lambda x, v, e, r: 0.0, noisy=False, delay_steps=0, motor_lag=0.0)
check(fell is not None, "a robot with no controller falls and is reported")

# 7. Noise model: same seed gives the same result; encoder step size is right.
a, _ = dg.simulate(0.10, dg.lqr_controller(K), seed=3)
b, _ = dg.simulate(0.10, dg.lqr_controller(K), seed=3)
check(a["tilt"] == b["tilt"], "same seed gives identical results")
check(abs(dg.COUNT_M - 0.000218) < 2e-6, "one encoder count is about 0.218 mm of travel")

# 8. Header block format.
block = dg.header_block(K)
check("BALANCE_LQR_K0 (-3.16227766f)" in block and "BALANCE_LQR_K3" in block, "header block matches controller.h format")

# 8b. A weak motor loses: with only 0.5 m/s^2 of wheel acceleration the robot falls.
ups = dg.survival(0.062, dg.lqr_controller(dg.lqr_gains(0.062)), runs=5, max_accel=0.5)[0]
check(ups == 0, "with 0.5 m/s^2 of wheel acceleration the robot falls in every run")

# 8c. Latency cliff: 15 ms delay with 30 ms motor lag is fine, 30 ms delay with 50 ms lag is not.
ok_up = dg.survival(0.062, dg.lqr_controller(dg.lqr_gains(0.062)), runs=5, delay_steps=3, motor_lag=0.03)[0]
bad_up = dg.survival(0.062, dg.lqr_controller(dg.lqr_gains(0.062)), runs=5, delay_steps=6, motor_lag=0.05)[0]
check(ok_up == 5 and bad_up == 0, "latency: 15 ms delay and 30 ms lag stays up, 30 ms delay and 50 ms lag falls")

# 9. Matched gains survive noisy runs at the planned heights.
for L in (0.062, 0.10):
    ups = dg.survival(L, dg.lqr_controller(dg.lqr_gains(L)), runs=10)[0]
    check(ups >= 9, f"matched LQR stays up in at least 9 of 10 noisy runs at L = {L} (got {ups})")
    
print()
print("ALL TESTS PASSED" if failures == 0 else "TESTS FAILED", f"({failures} failures)")
raise SystemExit(1 if failures else 0)