import numpy as np
import matplotlib.pyplot as plt
import joblib
import csv

import generation
import constraints

gp_lift = joblib.load("lift_dvs.joblib")
gp_drag = joblib.load("drag_dvs.joblib")


# Read the CSV file
with open("cfd_data_testing.csv", "r") as file:
    reader = csv.reader(file)
    filtered_samples_test = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers
filtered_samples_test = np.array(filtered_samples_test, dtype = float)
print("CSV data loaded:")

lift_test = filtered_samples_test[:,-2]
drag_test = filtered_samples_test[:,-1]

filtered_samples_test = filtered_samples_test[:, 0:18]

num_factors = len(filtered_samples_test[0])
for i in range (len(filtered_samples_test)): 
    zeros = np.count_nonzero(filtered_samples_test[i] == 0)
    num_slats = num_factors - zeros - 6
    frame = np.copy(filtered_samples_test[i,num_factors-zeros-5:num_factors-zeros])
    slat_length = filtered_samples_test[i,num_slats]

    filtered_samples_test[i,num_slats:] = 0 # reset everything but slat angles
    filtered_samples_test[i,num_factors-5:] = frame # add back frame
    filtered_samples_test[i,num_factors-6] = slat_length


for i in range(0,len(filtered_samples_test)):

    test_point = filtered_samples_test[i]
    lift = lift_test[i]
    drag = drag_test[i]

    [airfoil_x, airfoil_y], te_slats_used, frame_control, _, _, _ = generation.opt_generate(test_point)
    inter = generation.add_points([airfoil_x, airfoil_y], 54)
    input = np.array([j for i in zip(inter[0],inter[1]) for j in i])
    input = input.reshape(1,-1)

    input = test_point.reshape(1,-1)

    drag_pred, drag_sd = gp_drag.predict(input, return_std = True)
    lift_pred, lift_sd = gp_lift.predict(input, return_std = True)
    #print('Prediction: ' + str(drag_pred) + ', Actual: ' + str(drag), ', SD: ' + str(drag_sd))
    print('Prediction: ' + str(lift_pred) + ', Actual: ' + str(lift),  ', SD: ' + str(lift_sd))