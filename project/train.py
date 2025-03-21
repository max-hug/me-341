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
num_factors = len(filtered_samples[0])

for i in range (len(filtered_samples)): 
    zeros = np.count_nonzero(filtered_samples[i] == 0)
    num_slats = num_factors - zeros - 6
    frame = np.copy(filtered_samples[i,num_factors-zeros-5:num_factors-zeros])
    slat_length = filtered_samples[i,num_slats]

    filtered_samples[i,num_slats:] = 0 # reset everything but slat angles
    filtered_samples[i,num_factors-5:] = frame # add back frame
    filtered_samples[i,num_factors-6] = slat_length

# coords = []
# for trial in filtered_samples:
#     [airfoil_x, airfoil_y], te_slats_used, frame_control, _, _, _ = generation.opt_generate(trial)
#     inter = generation.add_points([airfoil_x, airfoil_y], 54)
#     input = np.array([j for i in zip(inter[0],inter[1]) for j in i])
#     coords.append(input)
#X_train = coords


X_train = filtered_samples

# Define Kriging (Gaussian Process) model
#kernel1 = C(1.0) * ExpSineSquared(length_scale=2,  length_scale_bounds= (1,1e8) ,periodicity=1)  # RBF Kernel with automatic tuning
kernel1 = C(1000.0) * RBF(length_scale=1.5, length_scale_bounds=(.5,1e6))  # RBF Kernel with automatic tuning
kernel2 = C(1000.0) * RBF(length_scale=1.5,  length_scale_bounds= (.5,1e6))  # RBF Kernel with automatic tuning

gp_lift = GaussianProcessRegressor(kernel=kernel1, n_restarts_optimizer=50, alpha=1e-6, normalize_y=True, random_state=1)
gp_drag = GaussianProcessRegressor(kernel=kernel2, n_restarts_optimizer=50, alpha=1e-6, normalize_y=True, random_state=1)
# Train Kriging model
gp_lift.fit(X_train, lift)
gp_drag.fit(X_train, drag)

joblib.dump(gp_lift, "lift_dvs.joblib")
joblib.dump(gp_drag, "drag_dvs.joblib")

print("Models Trained")
