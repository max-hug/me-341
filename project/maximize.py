from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import numpy as np
from scipy.optimize import minimize
import joblib
import generation
import csv
import constraints
import matplotlib.pyplot as plt


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
    gp_input_dim = 18

    x = np.insert(x, len(x) - 6, np.zeros(18-len(x)))

    input = np.concatenate((x, np.zeros(18-len(x)))).reshape(1,-1)
    lift_pred, lift_sd = gp_lift.predict(input, return_std=True)
    drag_pred, drag_sd = gp_drag.predict(input, return_std=True)

    res = -1*(lift_pred-0.0*lift_sd)/(drag_pred+0.0*drag_sd)  # Negative because we minimize in SciPy

    if abs(drag_pred) < 250 or abs(lift_pred) > 50000:
        res = 0

    if abs(res) > 150:
        res = 0

    return res

ics = load_ics("cfd_data.csv")
gp_lift = joblib.load("lift_dvs.joblib")
gp_drag = joblib.load("drag_dvs.joblib")

constraints_list = [{'type': 'ineq', 'fun': constraints.angle_opt}, 
                    {'type': 'ineq', 'fun': constraints.frame_opt}, 
                    {'type': 'ineq', 'fun': constraints.te_slat_opt}, 
                    {'type': 'ineq', 'fun': constraints.max_reflex_opt, 'hess':0}, 
                    {'type': 'ineq', 'fun': constraints.min_reflex_opt, 'hess':0}, 
                    {'type': 'ineq', 'fun': constraints.connector_length_opt}]


# Perform optimization using surrogate model
print("Beginning Optimization")
optimal_obj = [0] * 8  # 5-12 + 1
optimal_vals = [0] * 8
optimal_ics = [0] * 8
ics_obj = [0] * 8 

print(optimal_obj)

history = []
for IC in range(379,380):
    
    num_slats = len(ics[IC]) - np.count_nonzero(ics[IC] == 0) - 6

    lower_bound = np.array([2.15] * num_slats + [0.7/num_slats] + [0, 0.5, 0.12, 0.1, 0.01])
    upper_bound = np.array([3.5] + [np.pi] * (num_slats-1) + [1.25/num_slats] + [0.5, 1, 0.3, 0.25, 0.2]) 
    bounds = zip(lower_bound, upper_bound)

    ic = ics[IC][0:num_slats+6]
    history_new = []
    def call(*,intermediate_result):
        history_new.append(-1*intermediate_result.fun)

    # SLSQP, COBYQA, trust-constr
    result = minimize(surrogate_objective, ic, method='COBYQA', bounds=bounds, constraints = constraints_list, tol = 1e-3, options={'maxiter':10000}, callback = call)
    print(result.message)

    val = surrogate_objective(result.x)
    if  val < optimal_obj[num_slats-5]:
        optimal_obj[num_slats-5] = val
        optimal_vals[num_slats-5] = result.x
        optimal_ics[num_slats-5] = ic
        ics_obj[num_slats-5] = surrogate_objective(ic)
        print(optimal_obj)
        history = np.array(history_new)

plt.figure()
plt.title('Objective Function Convergence')
plt.xlabel('Iteration')
plt.ylabel('Objective Function Value')
plt.grid()
plt.plot(history)
plt.show()




output = [0] * len(optimal_obj)

generation.opt_generate(optimal_vals[-1], True)

for i in range (0,len(output)):
    output[i] = [optimal_obj[i], *optimal_vals[i], ics_obj[i], *optimal_ics[i]]

# Writing to a CSV file
#with open("exaustive.csv", "w", newline="") as file:
#    writer = csv.writer(file)
#    writer.writerows(output)

#print("CSV file created successfully!")


