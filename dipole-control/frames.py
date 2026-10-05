"""Planar camera/model transforms shared by GUI and control worker.

R maps model XY to camera world XY and may include a reflection. Positions
use mm on the camera side and m on the solver side; vectors retain their units
and Z component. The existing model plane remains z=0, without height calibration.
"""

import numpy as np
from numpy.typing import ArrayLike, NDArray


def cam_to_model_pos(
    pos_mm: ArrayLike, R: ArrayLike, offset_mm: ArrayLike
) -> NDArray[np.float64]:
    """Return model position in m from camera XY in mm.

    offset_mm is the model origin expressed in camera world XY. Thus
    p_model_xy_mm = R.T @ (p_camera_xy_mm - offset_mm). The identity/default
    case retains the previous conversion order, including signed zero.
    """
    xy = np.asarray(pos_mm, dtype=np.float64)[:2]
    rotation = np.asarray(R, dtype=np.float64)
    offset = np.asarray(offset_mm, dtype=np.float64)
    if not (np.array_equal(rotation, np.eye(2)) and not np.any(offset)):
        xy = rotation.T @ (xy - offset)
    return np.array([xy[0], xy[1], 0.0]) * 1e-3


def cam_to_model_vec(v3: ArrayLike, R: ArrayLike) -> NDArray[np.float64]:
    """Rotate/reflect camera vector XY into model XY; keep Z and units."""
    out = np.asarray(v3, dtype=np.float64).copy()
    out[:2] = np.asarray(R, dtype=np.float64).T @ out[:2]
    return out


def model_to_cam_vec(v3: ArrayLike, R: ArrayLike) -> NDArray[np.float64]:
    """Rotate/reflect model vector XY into camera XY; keep Z and units."""
    out = np.asarray(v3, dtype=np.float64).copy()
    out[:2] = np.asarray(R, dtype=np.float64) @ out[:2]
    return out
