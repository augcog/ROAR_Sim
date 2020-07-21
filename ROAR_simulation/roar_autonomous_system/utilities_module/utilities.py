import numpy as np


def png_to_depth(im: np.array) -> np.array:
    """
    Takes in an image read from cv2.imread(), whose output is simply a numpy
    array,
    turn it into a depth image according to carla's method of
    (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
    Args:
        im: input image, read from cv2.imread()
    Returns:
        depth image
    """
    # im = im.astype(np.float64)
    normalized_depth = np.dot(im[:, :, :3], [1, 256, 65536.0])
    normalized_depth /= 16777215.0
    return normalized_depth
