#!/usr/bin/env python3
"""Drivetrain limits for robot.mayhem.json, from the motor, gearing and robot mass.

usage: robot_limits.py [--mass-lb 150] [--cof 1.1] [--efficiency 0.9] [--battery 11.5] [--write]

Numbers come from TunerConstants.h (gear ratio, wheel radius, slip/stator current, module
positions) and the Kraken X60 motor curve. --write updates robot.constraints in robot.mayhem.json.
"""
import argparse, json, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LB = 0.45359237
G = 9.81

# Kraken X60 (non-FOC; the drive request does not enable FOC): 12 V curve
FREE_RPM, STALL_TORQUE, STALL_CURRENT = 6000.0, 7.09, 366.0
KT = STALL_TORQUE / STALL_CURRENT  # N*m/A
R_MOTOR = 12.0 / STALL_CURRENT  # ohm

# TunerConstants.h
GEAR = 5.902777777777778
WHEEL_R = 2 * 0.0254
SLIP_CURRENT = 70.0  # A, drive stator limit
MODULE_R = 10.343 * 0.0254 * math.sqrt(2)  # center to module
SPEED_AT_12V = 5.23  # m/s, CTRE's estimate in TunerConstants


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mass-lb", type=float, default=150.0,
                    help="robot + bumpers + battery at max weight")
    ap.add_argument("--cof", type=float, default=1.1, help="wheel tread static friction")
    ap.add_argument("--efficiency", type=float, default=0.9, help="gear train efficiency")
    ap.add_argument("--battery", type=float, default=11.5, help="battery volts under load")
    ap.add_argument("--size", type=float, default=0.813, help="bumper-to-bumper square, m")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()

    m = a.mass_lb * LB
    # Square frame of uniform density plus heavier bumpers at the edge: ~1.2x a uniform plate.
    moi = 1.2 * m * (2 * a.size**2) / 12
    free_12 = FREE_RPM / 60 / GEAR * 2 * math.pi * WHEEL_R
    v_free = min(free_12, SPEED_AT_12V) * a.battery / 12
    # Leave ~10% voltage for feedback so the follower can still correct at top speed.
    v_max = 0.9 * v_free

    wheel_force = SLIP_CURRENT * KT * GEAR * a.efficiency / WHEEL_R  # per module
    a_current = 4 * wheel_force / m
    a_traction = a.cof * G
    a_low = min(a_current, a_traction)
    # Back-EMF: the speed above which the battery can no longer push the full stator current.
    v_knee = v_free * (1 - SLIP_CURRENT * R_MOTOR / a.battery)
    # The follower uses one acceleration limit at all speeds; use the value at ~70% of top speed.
    v_eval = 0.7 * v_max
    i_avail = min(SLIP_CURRENT, (a.battery - a.battery * v_eval / v_free) / R_MOTOR)
    a_max = min(a_low, 4 * i_avail * KT * GEAR * a.efficiency / WHEEL_R / m)

    w_free = v_max / MODULE_R
    # Rotation and translation share the same wheels; keep spin to ~60% of the wheel speed.
    w_max = 0.6 * w_free
    alpha_max = 0.6 * 4 * wheel_force * MODULE_R / moi

    out = {
        "maxVelocity": round(v_max, 2),
        "maxAcceleration": round(a_max, 2),
        "maxAngularVelocity": round(w_max, 2),
        "maxAngularAcceleration": round(alpha_max, 1),
    }
    print(f"mass {m:.1f} kg ({a.mass_lb:.0f} lb), MOI ~{moi:.1f} kg*m^2, module radius {MODULE_R:.3f} m")
    print(f"free speed: {free_12:.2f} m/s motor curve, {SPEED_AT_12V} m/s CTRE estimate -> "
          f"{v_free:.2f} m/s at {a.battery} V")
    print(f"accel: current-limited {a_current:.2f}, traction {a_traction:.2f}, "
          f"at {v_eval:.1f} m/s {a_max:.2f} m/s^2 (full current up to ~{v_knee:.1f} m/s)")
    print("constraints:", json.dumps(out))
    if a.write:
        p = ROOT / "robot.mayhem.json"
        d = json.loads(p.read_text())
        d["robot"]["constraints"].update(out)
        for r in d["routines"]:
            r.setdefault("constraints", {}).update(out)
        d["revision"] += 1
        p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
