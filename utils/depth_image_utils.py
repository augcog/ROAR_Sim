import cv2
import numpy as np
from pathlib import Path
import os


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
    print(
        f"Stat for resulting depth image: \nShape:{np.shape(normalized_depth)} | Min: {np.amin(normalized_depth)} | Max: {np.amax(normalized_depth)}")
    return normalized_depth


def clip_depth_to(depth_img: np.array, distance: float) -> np.array:
    """
    Clip the depth image to distance. Any values greater than distance will be set to 0
    Args:
        depth_img: input depth image
        distance: desired distance, in meters correspond to real world distance

    Returns:
        clipped depth image
    """
    assert 0 < distance < 1, "Your input distance has to be between 0 and 1"
    depth_img[depth_img > distance] = 0
    print(
        f"Stat for clipped depth image: \nShape:{np.shape(depth_img)} | Min: {np.amin(depth_img)} | Max: {np.amax(depth_img)}")
    return depth_img


def apply_normal_distortion(depth_img: np.array, max_depth: float = 1.0) -> np.array:
    """
    Apply normal distribution, and then clip it because we know that it can only go from 0 to max_depth

    Args:
        max_depth: maximum depth
        depth_img: input depth image

    Returns:
        resulting distorted image
    """
    normal_distorted = depth_img + np.random.normal(0, np.amax(depth_img), depth_img.shape)
    return clip_depth_to(normal_distorted, max_depth)


def apply_uniform_distortion(depth_img: np.array, max_depth: float = 1.0) -> np.array:
    """
    Apply uniform distribution, and then clip it because we know that it can only go from 0 to max_depth

    Args:
        max_depth: maximum depth
        depth_img: input depth image

    Returns:
        resulting distorted image
    """
    uniform_distorted = depth_img + np.random.uniform(low=0, high=1, size=np.shape(depth_img))
    return clip_depth_to(uniform_distorted, max_depth)


def visualize_img(img, duration=5000, title="Image"):
    cv2.imshow(title, img)
    cv2.waitKey(duration)


if __name__ == '__main__':
    depth_file_path = Path(os.getcwd()).parent / "data" / "output" / "front_depth" / "depth-282.png"
    assert depth_file_path.exists(), f"The file {depth_file_path} does not exist, check file path plz"
    original = cv2.imread(depth_file_path.as_posix())

    distance = 0.9

    clipped = png_to_depth(original)

    clipped = clip_depth_to(clipped, distance=distance)

    normal_distorted = apply_normal_distortion(clipped, distance)

    uniform_distorted = apply_uniform_distortion(clipped, distance)

    cv2.imshow("original", original)
    cv2.imshow("Clipped", clipped)
    cv2.imshow("Normally Distorted", normal_distorted)
    cv2.imshow("Uniformally Distorted", uniform_distorted)

    cv2.waitKey(10000)

    cv2.destroyAllWindows()
