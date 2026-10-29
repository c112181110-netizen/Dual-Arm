"""Validation and sampling shared by the real and simulated controller."""
import bisect
import math


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def validate_trajectory(trajectory, names, calibration):
    if len(trajectory.joint_names) != len(names) or set(trajectory.joint_names) != set(names):
        raise ValueError('Trajectory must contain exactly the four configured arm joints')
    if not trajectory.points:
        raise ValueError('Empty trajectory')
    indices = [trajectory.joint_names.index(n) for n in names]
    points, times, previous = [], [], -1.0
    for p in trajectory.points:
        t = seconds(p.time_from_start)
        if t < 0 or t <= previous:
            raise ValueError('Trajectory times must be non-negative and strictly increasing')
        if len(p.positions) != 4:
            raise ValueError('Four positions required at every trajectory point')
        if p.effort:
            raise ValueError('CM530 does not support trajectory effort commands')
        for array in [p.positions, p.velocities, p.accelerations]:
            if array and (len(array) != 4 or not all(math.isfinite(v) for v in array)):
                raise ValueError('Invalid trajectory values')
        q = [p.positions[i] for i in indices]
        calibration.to_raw(q)
        points.append(q)
        times.append(t)
        previous = t
    return times, points


def sample(times, points, elapsed):
    """Piecewise linear resampling; positions are the only firmware command."""
    if elapsed >= times[-1]:
        return list(points[-1])
    if elapsed <= times[0]:
        return list(points[0])
    i = bisect.bisect_right(times, elapsed) - 1
    ratio = (elapsed - times[i]) / (times[i + 1] - times[i])
    return [a + ratio * (b - a) for a, b in zip(points[i], points[i + 1])]
