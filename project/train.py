from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ExpSineSquared, RBF, Matern, ConstantKernel as C
import numpy as np
from scipy.optimize import minimize
import joblib
import generation
import matplotlib.pyplot as plt

import csv

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
    [airfoil_x, airfoil_y], te_slats_used, frame_control, _, _, _ = generation.opt_generate(trial)
    inter = generation.add_points([airfoil_x, airfoil_y], 54)
    input = np.array([j for i in zip(inter[0],inter[1]) for j in i])
    coords.append(input)

X_train = coords

# Define Kriging (Gaussian Process) model
#kernel1 = C(1.0) * ExpSineSquared(length_scale=2,  length_scale_bounds= (1,1e8) ,periodicity=1)  # RBF Kernel with automatic tuning
kernel1 = C(100.0) * RBF(length_scale=1.5,  length_scale_bounds= (1.05,1e6))  # RBF Kernel with automatic tuning
kernel2 = C(1000.0) * RBF(length_scale=1.5,  length_scale_bounds= (.5,1e6))  # RBF Kernel with automatic tuning

gp_lift = GaussianProcessRegressor(kernel=kernel1, n_restarts_optimizer=30, alpha=1e-6, normalize_y=False, random_state=1)
gp_drag = GaussianProcessRegressor(kernel=kernel2, n_restarts_optimizer=50, alpha=1e-3, normalize_y=True, random_state=1)
# Train Kriging model
gp_lift.fit(X_train, lift)
gp_drag.fit(X_train, drag)

joblib.dump(gp_lift, "lift_coords.joblib")
joblib.dump(gp_drag, "drag_coords.joblib")

# Read the CSV file
with open("cfd_data_test_points.csv", "r") as file:
    reader = csv.reader(file)
    filtered_samples_test = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers
filtered_samples_test = np.array(filtered_samples_test, dtype = float)
print("CSV data loaded:")

lift_test = filtered_samples_test[:,-2]
drag_test = filtered_samples_test[:,-1]

filtered_samples_test = filtered_samples_test[:, 0:18]

for i in range(0,len(filtered_samples_test)):

    test_point = filtered_samples_test[i]
    lift = lift_test[i]
    drag = drag_test[i]

    [airfoil_x, airfoil_y], te_slats_used, frame_control, _, _, _ = generation.opt_generate(test_point)
    inter = generation.add_points([airfoil_x, airfoil_y], 54)
    input = np.array([j for i in zip(inter[0],inter[1]) for j in i])
    input = input.reshape(1,-1)

    drag_pred, _ = gp_drag.predict(input, return_std = True)
    #lift_pred, _ = gp_lift.predict(input, return_std = True)
    print('Prediction: ' + str(drag_pred) + ', Actual: ' + str(drag))
    #print('Prediction: ' + str(lift_pred) + ', Actual: ' + str(lift))
