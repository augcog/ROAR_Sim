from typing import Any

from ROAR.roar_autonomous_system.perception_module.detector import Detector
import cv2


class FloodfillLaneDetector(Detector):
    def run_step(self) -> Any:
        if self.agent.front_rgb_camera.data is not None:
            curr_img = self.agent.front_rgb_camera.data.copy()
            seed_point = (640 // 2, 400)
            cv2.floodFill(image=curr_img,
                          seedPoint=seed_point,
                          newVal=(255, 0, 0),
                          loDiff=(3, 3, 3),
                          upDiff=(3, 3, 3),
                          mask=None)
            result = cv2.circle(img=curr_img, center=seed_point, radius=10,
                                color=(0, 0, 255), thickness=-1)
            cv2.imshow("floodfill", result)
            cv2.waitKey(1)
