import numpy as np
from pyDOE3 import lhs
import constraints
import generation

num_slats = 14
slat_length = 0.05
lower_bounds = np.array([0.5] * num_slats + [0] * 5 + [-20])  # lower bounds
upper_bounds = np.array([3.14159] * num_slats + [1.0] * 5 + [20])  # upper bounds

num_factors = len(lower_bounds)  # 20 factors
num_samples = len(lower_bounds) * 10

lhs_samples = lhs(num_factors, samples=num_samples)

doe_samples = lower_bounds + (upper_bounds - lower_bounds) * lhs_samples


def constraints_combined(doe_sample, num_factors, slat_length, num_slats):
    frame_x_control = [0, doe_sample[num_factors-6], doe_sample[num_factors-5], 1]
    frame_y_control = [doe_sample[num_factors-4], doe_sample[num_factors-3], doe_sample[num_factors-2], 0]
    frame_control = [frame_x_control, frame_y_control]

    #print(frame_control)
    [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, doe_sample[0:num_slats], 56, False)
    angle = constraints.angle_constraint([airfoil_x, airfoil_y])
    frame = constraints.frame_constraint([airfoil_x, airfoil_y], frame_control)
    slat = constraints.te_slat_constraint(te_slats_used, doe_sample[0:num_slats])
    if angle and frame and slat:
        print("valid")
        print(doe_sample)
        [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, doe_sample[0:num_slats], 56, True)
    return (angle and frame and slat)


# Step 5: Apply constraint filtering
filtered_samples = []
for i in range(len(doe_samples)):
    if(constraints_combined(doe_samples[i],num_factors,slat_length,num_slats)):
        filtered_samples.append(doe_samples[i])

# Step 6: If too many points are removed, resample until enough valid points are found
while len(filtered_samples) < num_samples:
    new_sample = lower_bounds + (upper_bounds - lower_bounds) * lhs(num_factors, num_samples)
    new_sample = new_sample[0]
    if constraints_combined(new_sample, num_factors, slat_length, num_slats):
        filtered_samples.append(new_sample)


