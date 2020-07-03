import cv2
from pathlib import Path
import os
import cv2
import numpy as np
import glob


def frames_to_video(folder_path: Path, file_suffix=".png", fps=10):
    assert file_suffix == ".png" or file_suffix == ".jpg" or file_suffix == "jpeg", "Files other than png and jpg are not supported"
    files = glob.glob(folder_path.as_posix() + f"/*{file_suffix}")
    files.sort(key=os.path.getmtime)

    img_array = []
    for filename in files:
        img = cv2.imread(filename)
        # print(img, filename)
        try:
            height, width, _ = img.shape
        except:
            height, width = img.shape
        size = (width, height)
        img_array.append(img)

    # print(np.shape(img_array[0]))
    out = cv2.VideoWriter((folder_path / 'result.avi').as_posix(), cv2.VideoWriter_fourcc(*'DIVX'), fps, size)

    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()


if __name__ == '__main__':
    foler_path = Path(os.getcwd()).parent / "data" / "output" / "front_depth"
    frames_to_video(foler_path, file_suffix=".npy")
