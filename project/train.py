from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import numpy as np
from scipy.optimize import minimize
import joblib
import generation
import matplotlib.pyplot as plt

import csv

def quick_generate(x):
    zeros = np.count_nonzero(x == 0)
    num_slats = 12 - zeros
    num_factors = len(x)
    frame_x_control = [0, x[num_factors-zeros-5], x[num_factors-zeros-4], 1]
    frame_y_control = [x[num_factors-zeros-3], x[num_factors-zeros-2], x[num_factors-zeros-1], 0]
    frame_control = [frame_x_control, frame_y_control]
    slat_length = x[num_slats]

    #print(frame_control)
    [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, x[0:num_slats], False)
    return [airfoil_x, airfoil_y], te_slats_used, frame_control

# Read the CSV file
with open("cfd_data.csv", "r") as file:
    reader = csv.reader(file)
    
    filtered_samples = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers

filtered_samples = np.array(filtered_samples, dtype = float)
print("CSV data loaded:")

lift = filtered_samples[:,-2]

drag = filtered_samples[:,-1]

filtered_samples = filtered_samples[:, 0:18]

coords = []
for trial in filtered_samples:
    [airfoil_x, airfoil_y], te_slats_used, frame_control = quick_generate(trial)
    inter = []
    for i in range(len(airfoil_x)):
        inter.append(airfoil_x[i])
        inter.append(airfoil_y[i])
    while len(inter) != 54:
        avg_x = (inter[-2] + inter[-4])/2
        avg_y = (inter[-1] + inter[-3])/2
        inter.insert(-3, avg_x)
        inter.insert(-3, avg_y)
    inter = np.array(inter)
    coords.append(inter)

X_train = coords  

# Define Kriging (Gaussian Process) model
kernel1 = C(1.0) * RBF(length_scale=1.0)  # RBF Kernel with automatic tuning
gp_lift = GaussianProcessRegressor(kernel=kernel1, n_restarts_optimizer=10)

kernel2 = C(1.0) * RBF(length_scale=1.0)
gp_drag = GaussianProcessRegressor(kernel=kernel2, n_restarts_optimizer=10)
# Train Kriging model
gp_lift.fit(X_train, lift)
gp_drag.fit(X_train, drag)

joblib.dump(gp_lift, "lift_coords.joblib")
joblib.dump(gp_drag, "drag_coords.joblib")

coords = np.array(coords)
drag_pred, _ = gp_drag.predict(coords[10].reshape(1,-1), return_std = True)
print('Prediction: ' + str(drag_pred) + ', Actual: ' + str(drag[10]))

print("models trained")