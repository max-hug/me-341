from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import numpy as np
from scipy.optimize import minimize
import joblib
import generation
import csv
import constraints
from shapely.geometry import Point, Polygon

def find_angle(points):
    v1 = points[1]-points[0]
    v2 = points[2]-points[1]

    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    # Compute the cross product to determine the sign
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]

    sin_theta = cross_product / (magnitude_v1 * magnitude_v2)

    cos_theta = dot_product / magnitude_v1*magnitude_v2

    angle = np.asin(max(-1, min(1, sin_theta))) # Clamping for numerical stability

    return angle 

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

def angle_constraint(x):
    airfoil_coords, te_slats_used, frame_control = quick_generate(x)
    max_angle = -10

    for i in range(0, len(airfoil_coords[0])-2):
        points = np.array(list(zip(airfoil_coords[0][i:i+3], airfoil_coords[1][i:i+3])))
        angle = find_angle(points)
        max_angle = max(max_angle,angle)

    return (-max_angle) + 0.15

def frame_constraint(x):

    airfoil_coords, te_slats_used, frame_control = quick_generate(x)

    frame_control = np.array(frame_control)

    airfoil_x_filtered = airfoil_coords[0][airfoil_coords[0] < 0.2]
    airfoil_y_filtered = airfoil_coords[1][airfoil_coords[0] < 0.2]

    airfoil_x_filtered = np.append(airfoil_x_filtered, [1,1])
    airfoil_y_filtered = np.append(airfoil_y_filtered, [airfoil_y_filtered[-1], 0])

    polygon_points = np.column_stack([airfoil_x_filtered, airfoil_y_filtered])
    polygon = Polygon(polygon_points)

    t = np.linspace(0+1e-5, 1-1e-1, 20) # since its touching at the ends, dont start at them
    bezier_3 = lambda x, t: x[0]*(1-t)**3 + 3*x[1]*t*(1-t)**2 + 3*x[2]*t**2*(1-t)+x[3]*t**3
    frame_samples = np.column_stack([-1*bezier_3(frame_control[0], t), bezier_3(frame_control[1], t)])

    return (-1*sum(not polygon.contains(Point(p)) for p in frame_samples)/20.0) + 0.15 # 20 since 20 samples

def te_slat_constraint(x):
    airfoil_coords, slats_used, frame_control = quick_generate(x)
    slat_angles = x[0:12]
    num_slats = len(slat_angles)
    return 1.0*(slats_used-num_slats)/num_slats + 0.15

def max_reflex_constraint(x):
    slat_angles = x[0:12]
    return 3*np.pi/2 - sum(np.pi-np.array(slat_angles)) + 0.15

def min_reflex_constraint(x):
    slat_angles = x[0:12]
    return sum(np.pi-np.array(slat_angles)) - 3*np.pi/4 + 0.15

def connector_length(x):
    airfoil_coords, slats_used, frame_control = quick_generate(x)
    slat_angles = x[0:12]
    num_slats = len(slat_angles)
    slat_length = x[num_slats]
    point1 = np.array(airfoil_coords[0][num_slats], airfoil_coords[1][num_slats])
    point2 = np.array(airfoil_coords[0][num_slats+1], airfoil_coords[1][num_slats+1])
    dist = np.linalg.norm(point2-point1)
    return 6*slat_length - dist + 0.15

def constraints_combined(x):
    num_factors = len(x)
    num_slats = 12
    frame_x_control = [0, x[num_factors-5], x[num_factors-4], 1]
    frame_y_control = [x[num_factors-3], x[num_factors-2], x[num_factors-1], 0]
    frame_control = [frame_x_control, frame_y_control]
    slat_length = x[num_slats]
    
    tol = 1.5e-1

    #print(frame_control)
    [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, x[0:num_slats], False)
    angle = constraints.angle_constraint([airfoil_x, airfoil_y]) + tol
    frame = constraints.frame_constraint([airfoil_x, airfoil_y], frame_control) + tol
    slat = constraints.te_slat_constraint(te_slats_used, x[0:num_slats]) + tol
    min_reflex = constraints.min_reflex_constraint(x[0:num_slats]) + tol
    max_reflex = constraints.max_reflex_constraint(x[0:num_slats]) + tol
    connector_length = constraints.connector_length([airfoil_x, airfoil_y], x[0:num_slats], slat_length) + tol

    #frame = 1
    #angle = 1

    passed = angle > 0 and frame > 0 and slat > 0 and min_reflex > 0 and max_reflex > 0 and connector_length > 0

    
    #if passed:
        #print(angle, frame, slat, reflex)
    # print(doe_sample)
        #[airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, doe_sample[0:num_slats], 56, True)
    
    if passed: 
        return -1
    else:
        return 1

# Read the CSV file
with open("cfd_data.csv", "r") as file:
    reader = csv.reader(file)
    
    filtered_samples = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers

filtered_samples = np.array(filtered_samples, dtype = float)
print("CSV data loaded:")

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
    coords.append(inter)

X_train = filtered_samples   

gp_lift = joblib.load("lift_coords.joblib")
gp_drag = joblib.load("drag_coords.joblib")

coords = np.array(coords)

num_slats = 12

constraints_list = [{'type': 'ineq', 'fun': angle_constraint}, {'type': 'ineq', 'fun': frame_constraint}, {'type': 'ineq', 'fun': te_slat_constraint}, {'type': 'ineq', 'fun': max_reflex_constraint}, {'type': 'ineq', 'fun': min_reflex_constraint}, {'type': 'ineq', 'fun': connector_length}]
#constraints_list = [{'type': 'ineq', 'fun': constraints_combined}]

# Define the surrogate-based objective function (Cl/Cd Maximization)
def surrogate_objective(x):
    [airfoil_x, airfoil_y], te_slats_used, frame_control = quick_generate(x)
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
    lift_pred, _ = gp_lift.predict(inter.reshape(1,-1), return_std=True)
    drag_pred, _ = gp_drag.predict(inter.reshape(1,-1), return_std = True)
    return -1*lift_pred/drag_pred  # Negative because we minimize in SciPy

lower_bounds = np.array([2.15] * num_slats + [0.7/num_slats] + [0, 0.5, 0.12, 0.1, 0.01])
upper_bounds = np.array([3.5] + [np.pi] * (num_slats-1) + [1.25/num_slats] + [0.5, 1, 0.3, 0.25, 0.2]) 

bounds = []
for i in range(len(lower_bounds)):
    bounds.append([lower_bounds[i], upper_bounds[i]])

# Perform optimization using surrogate model
min_val = 0
for IC in range(175,200):
    result = minimize(surrogate_objective, X_train[IC], method='SLSQP', bounds=bounds, constraints = constraints_list, tol = 1e-4)
    val = surrogate_objective(result.x)
    if val < min_val:
        min_val = val
        optimal_design = result.x

print(surrogate_objective(optimal_design))
num_factors = len(optimal_design)
num_slats = 12
frame_x_control = [0, optimal_design[num_factors-5], optimal_design[num_factors-4], 1]
frame_y_control = [optimal_design[num_factors-3], optimal_design[num_factors-2], optimal_design[num_factors-1], 0]
frame_control = [frame_x_control, frame_y_control]
slat_length = optimal_design[num_slats]

#print(frame_control)
[airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, optimal_design[0:num_slats], True)





