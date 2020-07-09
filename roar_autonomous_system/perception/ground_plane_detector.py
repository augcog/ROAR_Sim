from roar_autonomous_system.perception.detector import Detector
import numpy as np
from typing import Optional
import cv2
from roar_autonomous_system.util.models import DepthData
from roar_autonomous_system.perception.utils import png_to_depth


class GroundPlaneDetector(Detector):
    def __init__(self,
                 sky_line_level: int = 310,
                 show: bool = False,
                 max_detectable_distance_threshold: float = 0.089,
                 min_caliberation_boundary: float = 0.01):
        super().__init__()
        self.sky_line_level = sky_line_level
        self.max_detectable_distance_threshold = max_detectable_distance_threshold
        self.min_caliberation_boundary = min_caliberation_boundary
        self._test_depth_img: Optional[np.asarray] = None
        self._predict_matrix: Optional[np.asarray] = None  # preds
        self.curr_depth_img: Optional[DepthData] = None
        self.show = show

    def run_step(self):
        """
        This function assumes that the function calling it will set the variable self.curr_depth_img

        In the first run of this function,
            it will remember the input depth image as self._test_depth_img
        In the second run of this function,
            it will calculate the prediction matrix
        In the preceding runs, it will use the prediction matrix to find ground plane

        Returns:
            None
        """

        if self.curr_depth_img is not None and self._predict_matrix is not None:
            # run a normal prediction on subsequent frames received
            # never modify the original data, this is of shape (WIDTH x HEIGHT x 3)
            depth_img = self.curr_depth_img.data.copy()
            depth_array = png_to_depth(depth_img)  # this turns it into 2D np array of shape (Width x Height)
            diff = np.abs(depth_array - self._predict_matrix)
            dets = (diff > self.max_detectable_distance_threshold)
            # dets is a 2D array of shape WidthxHeight of boolean. True = obstacle, False=otherwise
            depth_img[dets > 0] = 255
            if self.show:
                self.show_first_person_view(depth_img)
                # self.show_bird_eye_view(depth_img, dets)
        elif self.curr_depth_img is not None:
            if self._test_depth_img is None:
                # populate test image on the first frame received
                self._test_depth_img = png_to_depth(self.curr_depth_img.data)
                return
            else:
                # try calibrate on the second frame received
                xs = []
                data = []
                depth_array = png_to_depth(self.curr_depth_img.data)
                # depth_image = calibration image, grab from somewhere
                for i in range(self.sky_line_level, depth_array.shape[0]):
                    j = np.argmax(depth_array[i, :])
                    if depth_array[i][j] > self.min_caliberation_boundary:
                        xs.append(i)
                        data.append(depth_array[i][j])
                a, b, c, p, q = self.fit(
                    np.array(xs, dtype=np.float64),
                    np.array(data, dtype=np.float64)
                )
                test_image = self._test_depth_img
                pred_func = self.construct_f(a, b, c, p, q)
                rows = np.meshgrid(
                    np.arange(test_image.shape[1]), np.arange(test_image.shape[0])
                )[1]
                self._predict_matrix = pred_func(rows)
                return
        else:
            return None

    def recalibrate(self):
        """
        Force a recalibration of the ground plane prediction matrix

        Returns:
            None
        """
        self._test_depth_img = None
        self._predict_matrix = None

    def sync(self):
        pass

    @classmethod
    def show_first_person_view(cls, depth_image):
        """
        show the depth image
        Args:
            depth_image: Width x height x 3 shaped image

        Returns:

        """
        gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
        color = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        cv2.imshow('First Person', color)
        cv2.waitKey(1)

    def Sk(self, S_k_1, y_k, y_k_1, x_k, x_k_1):
        return S_k_1 + 0.5 * (y_k + y_k_1) * (x_k - x_k_1)

    def SSk(self, SS_k_1, S_k, S_k_1, x_k, x_k_1):
        return SS_k_1 + 0.5 * (S_k + S_k_1) * (x_k - x_k_1)

    def S(self, x, y):
        ret_S = [0]
        for k in range(1, len(x)):
            S_k = self.Sk(ret_S[k - 1], y[k], y[k - 1], x[k], x[k - 1])
            ret_S.append(S_k)
        return ret_S

    def SS(self, s, x):
        ret_SS = [0]
        for k in range(1, len(x)):
            SS_k = self.SSk(ret_SS[k - 1], s[k], s[k - 1], x[k], x[k - 1])
            ret_SS.append(SS_k)
        return ret_SS

    def F1(self, SS_k, y_k):
        return SS_k / y_k

    def F2(self, S_k, y_k):
        return S_k / y_k

    def F3(self, x_k, y_k):
        return (x_k ** 2) / y_k

    def F4(self, x_k, y_k):
        return x_k / y_k

    def F5(self, y_k):
        return 1 / y_k

    def construct_f(self, a, b, c, p, q):
        def f(x):
            # print(type(a), type(b), type(c), type(p), type(q))
            return a + b * np.exp(p * x) + c * np.exp(q * x)

        return f

    def fit(self, x, y):
        Sxy = self.S(x, y)
        SSxy = self.SS(Sxy, x)
        F1xy = self.F1(SSxy, y)
        F2xy = self.F2(Sxy, y)
        F3xy = self.F3(x, y)
        F4xy = self.F4(x, y)
        F5xy = self.F5(y)
        F = np.array([F1xy, F2xy, F3xy, F4xy, F5xy])
        f = np.array([np.sum(F1xy), np.sum(F2xy), np.sum(F3xy),
                      np.sum(F4xy), np.sum(F5xy)])
        F = F @ F.T
        A, B, C, D, E = np.linalg.inv(F) @ f
        pre_sqrt = np.clip(B ** 2 + 4 * A, 0, np.inf)  # edits 1

        p = 0.5 * (B + np.sqrt(pre_sqrt))
        q = 0.5 * (B - np.sqrt(pre_sqrt))
        G1 = 1 / y
        G2 = np.exp(p * x) / y
        G3 = np.exp(q * x) / y
        G = np.array([G1, G2, G3])
        G = G @ G.T
        g = np.array([np.sum(G1), np.sum(G2), np.sum(G3)])
        a, b, c = np.linalg.pinv(G) @ g  # edits 2
        return a, b, c, p, q

    # def show_bird_eye_view(self, depth_img, dets):
    #     """
    #     show the depth image
    #     Args:
    #         depth_img: Width x height x 3 shaped image
    #
    #     Returns:
    #         None
    #     """
    #     width, height, _ = np.shape(depth_img)
    #     width, height = 200, 200
    #     source_ppts = np.float32([(231, 325), (600, 325), (789, 383), (2, 352)])
    #     dest_ppts = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    #     perspective = cv2.getPerspectiveTransform(source_ppts, dest_ppts)
    #     warped = cv2.warpPerspective(depth_img, perspective, (width, height))
    #     cv2.imshow("warped", warped)
    #     cv2.waitKey(1)
    #
    #
    #
    #     pass
