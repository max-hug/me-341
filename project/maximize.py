from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import numpy as np
from scipy.optimize import minimize
import joblib
import generation
import csv
import constraints

def load_ics(file_name):
    # Read the CSV file
    with open("cfd_data.csv", "r") as file:
        reader = csv.reader(file)
        
        filtered_samples = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers

    filtered_samples = np.array(filtered_samples, dtype = float)
    print("CSV data loaded:")

    dvs = filtered_samples[:, 0:18]

    return np.array(dvs)

# Define the surrogate-based objective function (Cl/Cd Maximization)
def surrogate_objective(x):
    [airfoil_x, airfoil_y], te_slats_used, frame_control, _, _, _ = generation.opt_generate(x)
    inter = generation.add_points([airfoil_x, airfoil_y], 54)
    input = np.array([j for i in zip(inter[0],inter[1]) for j in i])
    input = input.reshape(1,-1)
    lift_pred, _ = gp_lift.predict(input, return_std=True)
    drag_pred, _ = gp_drag.predict(input, return_std=True)

    if abs(drag_pred) < 1e-6:
        res = 0
    else:
        res = -1*lift_pred/drag_pred  # Negative because we minimize in SciPy

    return res

ics = load_ics("cfd_data.csv")
gp_lift = joblib.load("lift_coords.joblib")
gp_drag = joblib.load("drag_coords.joblib")

constraints_list = [{'type': 'ineq', 'fun': constraints.angle_opt}, 
                    {'type': 'ineq', 'fun': constraints.frame_opt}, 
                    {'type': 'ineq', 'fun': constraints.te_slat_opt}, 
                    {'type': 'ineq', 'fun': constraints.max_reflex_opt}, 
                    {'type': 'ineq', 'fun': constraints.min_reflex_opt}, 
                    {'type': 'ineq', 'fun': constraints.connector_length_opt}]


# Perform optimization using surrogate model
print("Beginning Optimization")
min_val = 1
optimal_result = -1
optimal_ic = []

for IC in range(175,199):

    num_slats = len(ics[IC]) - np.count_nonzero(ics[IC] == 0) - 6
    lower_bound = np.array([2.15] * num_slats + [0.7/num_slats] + [0, 0.5, 0.12, 0.1, 0.01])
    upper_bound = np.array([3.5] + [np.pi] * (num_slats-1) + [1.25/num_slats] + [0.5, 1, 0.3, 0.25, 0.2]) 
    bounds = zip(lower_bound, upper_bound)

    # SLSQP, COBYQA, trust-constr
    result = minimize(surrogate_objective, ics[IC], method='trust-constr', bounds=bounds, constraints = constraints_list, tol = 1e-3, options={'maxiter':1000})
    print(result.message)

    val = surrogate_objective(result.x)
    if val < min_val:
        min_val = val
        optimal_result = result
        optimal_ic = ics[IC]

print(optimal_ic)

print(surrogate_objective(result.x))
print(surrogate_objective(optimal_ic))


generation.opt_generate(result.x, True)
generation.opt_generate(optimal_ic, True)





