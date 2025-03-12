from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import numpy as np
from scipy.optimize import minimize
import joblib

import csv

# Read the CSV file
with open("cfd_data.csv", "r") as file:
    reader = csv.reader(file)
    
    filtered_samples = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers

filtered_samples = np.array(filtered_samples, dtype = float)
print("CSV data loaded:")

lift = filtered_samples[:,-2]
print(lift)

drag = filtered_samples[:,-1]

filtered_samples = filtered_samples[:, 0:18]

X_train = filtered_samples   


# Define Kriging (Gaussian Process) model
kernel1 = C(1.0) * RBF(length_scale=np.ones(18))  # RBF Kernel with automatic tuning
gp_lift = GaussianProcessRegressor(kernel=kernel1, n_restarts_optimizer=10)

kernel2 = C(1.0) * RBF(length_scale=np.ones(18))
gp_drag = GaussianProcessRegressor(kernel=kernel2, n_restarts_optimizer=10)
# Train Kriging model
gp_lift.fit(X_train, lift)
gp_drag.fit(X_train, drag)

joblib.dump(gp_lift, "lift.joblib")
joblib.dump(gp_drag, "drag.joblib")

print("models trained")

