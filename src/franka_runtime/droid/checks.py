"""Runtime-side physical acceptance of a native DROID chunk, after contract binding checks.

Setpoint-lead semantics: DROID rows are joint-impedance setpoints that may lead the
measured state. Rejects, never clips:
- every row inside the joint range reduced by margins;
- row 0 leading the measured state by at most ``max_target_lead_rad`` per joint;
- consecutive rows implying at most ``max_joint_velocity_rad_s`` at the row period.
These are target-admission checks, not measured velocity/acceleration guarantees.
The mock has a braking model; the native impedance candidate has separate guards.
"""


def check_rows(rows, measured_q, limits, row_period_ns):
    """``rows`` are (q1..q7, closure) tuples. Returns the peak row-to-row speed per joint (rad/s)."""
    lower, upper, margin = limits["lower"], limits["upper"], limits["margin"]
    max_velocity, max_lead = limits["max_velocity"], limits["max_target_lead"]
    if len(measured_q) != 7 or not rows:
        raise ValueError("Measured state must have seven joints and the chunk at least one row")
    for index, row in enumerate(rows):
        for j, (v, lo, hi, m) in enumerate(zip(row[:7], lower, upper, margin)):
            if not lo+m <= v <= hi-m:
                raise ValueError(f"Row {index} joint {j+1} target {v:.4f} outside margin-reduced range [{lo+m:.4f}, {hi-m:.4f}]")
    for j, (target, measured) in enumerate(zip(rows[0][:7], measured_q)):
        if abs(target-measured) > max_lead[j]:
            raise ValueError(f"Row 0 joint {j+1} leads measured state by {abs(target-measured):.3f} rad > {max_lead[j]:.3f}")
    period_s = row_period_ns/1e9
    peak = [0.]*7
    for index, (previous, row) in enumerate(zip(rows, rows[1:]), start=1):
        for j, (a, b) in enumerate(zip(previous[:7], row[:7])):
            speed = abs(b-a)/period_s
            peak[j] = max(peak[j], speed)
            if speed > max_velocity[j]:
                raise ValueError(f"Row {index-1}→{index} joint {j+1} implies {speed:.3f} rad/s > limit {max_velocity[j]:.3f}")
    return peak
