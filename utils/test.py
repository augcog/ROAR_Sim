from depth_image_utils import png_to_depth
import numpy as np
from pathlib import Path
import cv2
import os
import time


def Sk(S_k_1, y_k, y_k_1, x_k, x_k_1):
    return S_k_1 + 0.5 * (y_k + y_k_1) * (x_k - x_k_1)


def SSk(SS_k_1, S_k, S_k_1, x_k, x_k_1):
    return SS_k_1 + 0.5 * (S_k + S_k_1) * (x_k - x_k_1)


def S(x, y):
    ret_S = [0]
    for k in range(1, len(x)):
        S_k = Sk(ret_S[k - 1], y[k], y[k - 1], x[k], x[k - 1])
        ret_S.append(S_k)
    return ret_S


def SS(s, x):
    ret_SS = [0]
    for k in range(1, len(x)):
        SS_k = SSk(ret_SS[k - 1], s[k], s[k - 1], x[k], x[k - 1])
        ret_SS.append(SS_k)
    return ret_SS


def F1(SS_k, y_k):
    return SS_k / y_k


def F2(S_k, y_k):
    return S_k / y_k


def F3(x_k, y_k):
    return (x_k ** 2) / y_k


def F4(x_k, y_k):
    return x_k / y_k


def F5(y_k):
    return 1 / y_k


def construct_f(a, b, c, p, q):
    def f(x):
        # print(type(a), type(b), type(c), type(p), type(q))
        return a + b * np.exp(p * x) + c * np.exp(q * x)

    return f


def fit(x, y):
    Sxy = S(x, y)
    SSxy = SS(Sxy, x)
    F1xy = F1(SSxy, y)
    F2xy = F2(Sxy, y)
    F3xy = F3(x, y)
    F4xy = F4(x, y)
    F5xy = F5(y)
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

if __name__ == '__main__':
    from depth_image_utils import png_to_depth
    import numpy as np
    from pathlib import Path
    import cv2
    import os
    import time
    depth_img_path = Path(os.getcwd()).parent / "data" / "output" / "front_depth" / "depth-1.png"
    depth_img = cv2.imread(depth_img_path.as_posix())
    depth_img = png_to_depth(depth_img)

    test_img_path = Path(os.getcwd()).parent / "data" / "output" / "front_depth" / "depth-2.png"
    test_img = cv2.imread(test_img_path.as_posix())
    test_img = png_to_depth(test_img)


    xs = []
    data = []
    depth_image = depth_img
    # depth_image = calibration image, grab from somewhere
    for i in range(310, depth_image.shape[0]):
        j = np.argmax(depth_image[i, :])
        if depth_image[i][j] > 0.01:
            xs.append(i)
            data.append(depth_image[i][j])
    a, b, c, p, q = fit(
        np.array(xs, dtype=np.float64),
        np.array(data, dtype=np.float64)
    )
    # Grab test image from somewhere

    test_image = test_img
    pred_func = construct_f(a, b, c, p, q)
    rows = np.meshgrid(
        np.arange(test_image.shape[1]), np.arange(test_image.shape[0])
    )[1]
    preds = pred_func(rows)

    print(np.shape(preds))
    # video_file_path = Path(os.getcwd()).parent / "output" / "front_depth" / "result.avi"
    # cap = cv2.VideoCapture('result.avi')
    # prevFrame = None
    # prevData = None
    # avg = []
    # while True:
    #     if cap.isOpened():
    #         ret, frame = cap.read()
    #         try:
    #             t1 = time.time()
    #             diff = np.abs(png_to_depth(frame) - preds)
    #             dets = (diff > 0.089)
    #
    #             frame[dets > 0] = 255
    #
    #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #             color = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    #             cv2.imshow('frame', color)
    #             t2 = time.time()
    #
    #             avg.append(t2 - t1)
    #         except Exception as e:
    #             print(e)
    #             break
    #     cap.release()
    #     cv2.destroyAllWindows()
