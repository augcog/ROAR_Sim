import cv2
from pathlib import Path
import os
fp = Path(os.getcwd()) / "data" / "output" / "front_rgb" / "front_rgb-1.png"
im = cv2.imread(fp.as_posix())
cv2.imshow("image", im)
cv2.waitKey(5000)
