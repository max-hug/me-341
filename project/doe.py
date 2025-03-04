import numpy as np
from pyDOE3 import lhs
import constraints
import generation

num_slats = 14
slat_length = 0.05
lower_bounds = np.array([0] * (num_slats+5) + [-20])  # lower bounds
upper_bounds = np.array([np.pi] * num_slats + [1.0] * 5 + [20])  # upper bounds

num_factors = len(lower_bounds)  # 20 factors
num_samples = len(lower_bounds) * 10

lhs_samples = lhs(num_factors, samples=num_samples)

doe_samples = lower_bounds + (upper_bounds - lower_bounds) * lhs_samples


def constraints_combined(doe_samples, num_factors, slat_length, num_slats):
    frame_x_control = [0, doe_samples[num_factors-6], doe_samples[num_factors-5], 1]
    frame_y_control = [doe_samples[num_factors-4], doe_samples[num_factors-3], doe_samples[num_factors-2], 0]
    frame_control = [frame_x_control, frame_y_control]
    [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, doe_samples[0:num_slats], 56, False)
    angle = constraints.angle_constraint([airfoil_x, airfoil_y]) >= 0 and constraints.angle_constraint([airfoil_x, airfoil_y]) <= np.pi
    frame = constraints.frame_constraint([airfoil_x, airfoil_y], frame_control) == 0
    slat = constraints.te_slat_constraint(te_slats_used, doe_samples[0:num_slats]) >= 0
    return (angle and frame and slat)


# Step 5: Apply constraint filtering
filtered_samples = np.array([x for x in doe_samples if constraints_combined(x,num_factors,slat_length, num_slats)])

# Step 6: If too many points are removed, resample until enough valid points are found
while len(filtered_samples) < num_samples:
    new_sample = lower_bounds + (upper_bounds - lower_bounds) * lhs(1, num_factors)
    if constraints_combined(new_sample[0], num_factors, slat_length, num_slats):
        filtered_samples = np.vstack((filtered_samples, new_sample))


