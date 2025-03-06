import numpy as np
import matplotlib.pyplot as plt

import generation
import constraints


#slat_angles = [1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
#slat_angles = [1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
#slat_angles = [1.9,2,2.9,2.8,2.95,3.1,3.1]
slat_angles = [1, 2]

slat_length = 0.9/len(slat_angles)

frame_x_control = [0, 0.6, 0.85, 1]
frame_y_control = [0.3, 0.15, 0.07, 0]

frame_control = [frame_x_control, frame_y_control]
print(frame_control)

[airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, slat_angles, True)

#print(constraints.angle_constraint([airfoil_x, airfoil_y]))
#print(constraints.frame_constraint([airfoil_x, airfoil_y], frame_control))
print(constraints.te_slat_constraint(te_slats_used, slat_angles))
print(len(airfoil_x))

export_airfoil = np.array([airfoil_x, airfoil_y]).T

np.savetxt('airfoil.txt', export_airfoil, delimiter=',', fmt='%.3f', header='', comments='')


