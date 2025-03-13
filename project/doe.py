import numpy as np
from pyDOE3 import lhs
import constraints
import generation
import csv

filtered_overall = []
for num_slats in range(5,11):
    lower_bounds = np.array([1] * num_slats + [0.6/num_slats] + [0, 0.5, 0.12, 0.1, 0.01])
    upper_bounds = np.array([3.5] + [np.pi] * (num_slats-1) + [1.25/num_slats] + [0.5, 1, 0.3, 0.25, 0.2])  # upper bounds

    #lower_bounds = np.array([0.5] * num_slats + [0] * 5 + [-20])  # lower bounds
    #upper_bounds = np.array([3.14159] * num_slats + [1.0] * 5 + [20])  # upper bounds

    num_factors = len(lower_bounds)  # 20 factors
    num_samples = 2

    lhs_samples = lhs(num_factors, samples=num_samples)

    doe_samples = lower_bounds + (upper_bounds - lower_bounds) * lhs_samples


    def constraints_combined(doe_sample, num_factors, num_slats):
        frame_x_control = [0, doe_sample[num_factors-5], doe_sample[num_factors-4], 1]
        frame_y_control = [doe_sample[num_factors-3], doe_sample[num_factors-2], doe_sample[num_factors-1], 0]
        frame_control = [frame_x_control, frame_y_control]
        slat_length = doe_sample[num_slats]
        
        tol = 1.5e-1

        #print(frame_control)
        [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, doe_sample[0:num_slats], False)
        angle = constraints.angle([airfoil_x, airfoil_y]) + tol
        frame = constraints.frame([airfoil_x, airfoil_y], frame_control) + tol
        slat = constraints.te_slat(te_slats_used, doe_sample[0:num_slats]) + tol
        min_reflex = constraints.min_reflex(doe_sample[0:num_slats]) + tol
        max_reflex = constraints.max_reflex(doe_sample[0:num_slats]) + tol
        connector_length = constraints.connector_length([airfoil_x, airfoil_y], doe_sample[0:num_slats], slat_length) + tol

        frame = 1
        #angle = 1

        passed = angle > 0 and frame > 0 and slat > 0 and min_reflex > 0 and max_reflex > 0 and connector_length > 0

        
        #if passed:
            #print(angle, frame, slat, reflex)
        # print(doe_sample)
            #[airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, doe_sample[0:num_slats], True)
        
        return passed


    # Step 5: Apply constraint filtering
    filtered_samples = []
    for i in range(len(doe_samples)):
        if(constraints_combined(doe_samples[i],num_factors,num_slats)):
            filtered_samples.append(doe_samples[i])

    # Step 6: If too many points are removed, resample until enough valid points are found
    while len(filtered_samples) < num_samples:
        new_sample = lower_bounds + (upper_bounds - lower_bounds) * lhs(num_factors, 1)
        new_sample = new_sample[0]
        if constraints_combined(new_sample, num_factors, num_slats):
            filtered_samples.append(new_sample)
    
    filtered_overall.append(filtered_samples)
    print(str(num_slats) + " is done")

# Save to CSV
with open("testing_data_points2.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerows(filtered_overall)

print("CSV file saved successfully!")
