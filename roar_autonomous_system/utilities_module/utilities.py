import numpy as np
from roar_autonomous_system.utilities_module.data_structures_models import Transform

def png_to_depth(im: np.array) -> np.array:
    """
    Takes in an image read from cv2.imread(), whose output is simply a numpy array,
    turn it into a depth image according to carla's method of
    (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
    Args:
        im: input image, read from cv2.imread()
    Returns:
        depth image
    """
    im = im.astype(np.float64)
    normalized_depth = np.dot(im[:, :, :3], [65536.0, 256.0, 1.0])
    normalized_depth /= 16777215.0
    return normalized_depth

def calculate_extrinsics_from_euler(transform: Transform):
    """
    Calculate extrinsics matrix
    http://planning.cs.uiuc.edu/node104.html
    Args:
        transform: contains info regarding location and rotation with respect to parent object

    Returns:

    """
    location = transform.location
    rotation = transform.rotation
    yaw, pitch, roll = rotation.yaw, rotation.pitch, rotation.roll
    tx, ty, tz = location.x, location.y, location.z
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.identity(4)
    matrix[0, 3] = tx
    matrix[1, 3] = ty
    matrix[2, 3] = tz
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix