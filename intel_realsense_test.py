import pyrealsense2
import numpy as np
import cv2
def is_rgb_frame(frame):
    return True if frame.is_video_frame() and frame.is_depth_frame() is False else False


def is_depth_frame(frame):
    return True if frame.is_video_frame() and frame.is_depth_frame() else False

pipe = pyrealsense2.pipeline()
profile = pipe.start()
try:
    while True:
        frames: pyrealsense2.composite_frame = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_array = cv2.cvtColor(np.array(color_frame.get_data()), cv2.COLOR_BGR2RGB)

        depth_frame = frames.get_depth_frame()
        depth_array = np.array(depth_frame.get_data())
        cv2.imshow("color_frame", color_array)
        cv2.imshow("depth_frame", depth_array)
        cv2.waitKey(1)
        # for frame in frames:
        #     if is_rgb_frame(frame):

finally:
    pipe.stop()



