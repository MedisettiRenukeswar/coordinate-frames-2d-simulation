import math
from typing import Tuple


def rotation_matrix_2d(theta_rad: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Create a 2D rotation matrix for angle theta (in radians).

    R = [[cosθ, -sinθ],
         [sinθ,  cosθ]]
    """
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return ((c, -s),
            (s,  c))


def apply_rotation_2d(R: Tuple[Tuple[float, float], Tuple[float, float]],
                      p: Tuple[float, float]) -> Tuple[float, float]:
    """
    Apply a 2D rotation matrix R to a point p.
    """
    x, y = p
    (r00, r01), (r10, r11) = R
    xr = r00 * x + r01 * y
    yr = r10 * x + r11 * y
    return xr, yr


def transform_point_2d(R: Tuple[Tuple[float, float], Tuple[float, float]],
                       t: Tuple[float, float],
                       p: Tuple[float, float]) -> Tuple[float, float]:
    """
    Apply a 2D rigid transform to a point.

    p' = R * p + t
    """
    xr, yr = apply_rotation_2d(R, p)
    tx, ty = t
    return xr + tx, yr + ty


def compose_transform_2d(
    R1: Tuple[Tuple[float, float], Tuple[float, float]],
    t1: Tuple[float, float],
    R2: Tuple[Tuple[float, float], Tuple[float, float]],
    t2: Tuple[float, float],
) -> Tuple[Tuple[Tuple[float, float], Tuple[float, float]], Tuple[float, float]]:
    """
    Compose two 2D transforms (R1, t1) and (R2, t2).

    Result T = T1 ∘ T2 such that:
    p' = R1 * (R2 * p + t2) + t1
       = (R1 * R2) * p + (R1 * t2 + t1)
    """
    (r1_00, r1_01), (r1_10, r1_11) = R1
    (r2_00, r2_01), (r2_10, r2_11) = R2

    # R = R1 * R2
    R = (
        (r1_00 * r2_00 + r1_01 * r2_10, r1_00 * r2_01 + r1_01 * r2_11),
        (r1_10 * r2_00 + r1_11 * r2_10, r1_10 * r2_01 + r1_11 * r2_11),
    )

    # t = R1 * t2 + t1
    t2x, t2y = t2
    t1x, t1y = t1
    tx = r1_00 * t2x + r1_01 * t2y + t1x
    ty = r1_10 * t2x + r1_11 * t2y + t1y
    t = (tx, ty)

    return R, t
