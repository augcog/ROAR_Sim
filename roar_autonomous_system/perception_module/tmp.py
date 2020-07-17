import numpy as np
import exponential_model as exp_model
xs = []
data = []
# depth_image = calibration image, grab from somewhere
for i in range(310, depth_image.shape[0]):
    j = np.argmax(depth_image[i,:])
    if depth_image[i][j] > 0.01:
        xs.append(i)
        data.append(depth_image[i][j])
a, b, c ,p, q = exp_model.fit(
    np.array(xs, dtype=np.float64),
    np.array(data, dtype=np.float64)
)
# Grab test image from somewhere
pred_func = exp_model.construct_f(a, b, c, p, q)
rows = np.meshgrid(
    np.arange(test_image.shape[1]), np.arange(test_image.shape[0])
)[1]
preds = pred_func(rows)







