
import math
from typing import List, Tuple

def clamp_int(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, x))


def clamp_float(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_value(pulse_us: float, freq_hz: float) -> int:
    """Convert pulse width (us) to PCA9685 OFF counter value (0..4095)."""
    period_us = 1_000_000.0 / float(freq_hz)
    value = int(round(4095.0 * (float(pulse_us) / period_us)))
    return max(0, min(4095, value))

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def normalize_xy(x_cmd: float, y_cmd: float) -> Tuple[float, float]:
    # Keep diagonals consistent
    mag = math.sqrt(x_cmd * x_cmd + y_cmd * y_cmd)
    if mag > 1.0:
        x_cmd /= mag
        y_cmd /= mag
    return x_cmd, y_cmd

def mix_xy_yaw_to_4(u_x: float, u_y: float, u_yaw: float) -> List[float]:
    """
    Inputs in [-1,1]. Output 4 thruster commands in [-1,1] after desaturation.
    Convention:
      u_y = forward (+)
      u_x = right   (+)
      u_yaw = yaw right (+)
    """
    t1 = +u_y - u_x - u_yaw
    t2 = +u_y + u_x + u_yaw
    t3 = +u_y + u_x - u_yaw
    t4 = +u_y - u_x + u_yaw

    m = max(abs(t1), abs(t2), abs(t3), abs(t4), 1.0)
    return [t1 / m, t2 / m, t3 / m, t4 / m]

def u_to_pwm_us(u: float, neutral: float = 1500.0, span: float = 400.0) -> float:
    """
    Map u in [-1,1] -> PWM us in [neutral-span, neutral+span]
    Default: 1500 +/- 400 -> [1100,1900]
    """
    u = clamp(u, -1.0, 1.0)
    return neutral + span * u

def axes_to_pwm_raw(axes0: float, axes1: float, axes3: float, freq_hz: float = 50.0, span: float = 400.0) -> List[float]:
    # clamp
    x_cmd = clamp(-axes0, -1.0, 1.0)
    y_cmd = clamp(axes1, -1.0, 1.0)
    yaw_cmd = clamp(-axes3, -1.0, 1.0)

    # normalize XY for diagonals
    x_cmd, y_cmd = normalize_xy(x_cmd, y_cmd)

    t = mix_xy_yaw_to_4(x_cmd, y_cmd, yaw_cmd)
    return [us_to_value(u_to_pwm_us(ui, span=span), freq_hz) for ui in t]

def trigger_to_01(x: float) -> float:
    x = clamp(x, -1.0, 1.0)
    return (1.0 - x) * 0.5

def desaturate(us: List[float]) -> List[float]:
    m = max(1.0, max(abs(u) for u in us))
    return [u / m for u in us]

def vertical_mix_pwm_us(axis_up: float, axis_down: float, axis_pitch: float, freq_hz: float = 50.0, span: float = 400.0) -> List[float]:
    t_up = trigger_to_01(axis_up)
    t_down = trigger_to_01(axis_down)

    print(t_up, t_down)

    # Heave command in [-1, 1]
    u_z = clamp(t_up - t_down, -1.0, 1.0)

    # Pitch command in [-1, 1]
    u_pitch = clamp(axis_pitch, -1.0, 1.0)

    # Mix (no roll)
    fl = u_z - u_pitch
    fr = u_z - u_pitch
    rl = u_z + u_pitch
    rr = u_z + u_pitch

    fr = -fr
    rl = -rl

    # Desaturate to keep within [-1,1] while preserving ratios
    t = desaturate([fr, fl, rr, rl])

    # Convert to PWM us
    return [us_to_value(u_to_pwm_us(ui, span = span), freq_hz) for ui in t]
