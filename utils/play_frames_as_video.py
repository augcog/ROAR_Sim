import cv2
from pathlib import Path
import os
import cv2
import numpy as np
import glob


def frames_to_video(folder_path: Path, file_suffix=".png"):
    files = glob.glob(folder_path.as_posix() + f"/*{file_suffix}")
    files.sort(key=os.path.getmtime)
    for filename in files:
        img = cv2.imread(filename) if file_suffix != ".npy" else np.load(filename)
        # print(img, filename)
        cv2.imshow("img", img)
        cv2.waitKey(1)


if __name__ == '__main__':
    foler_path = Path(os.getcwd()).parent / "data" / "output" / "front_depth"
    frames_to_video(foler_path, file_suffix=".npy")
