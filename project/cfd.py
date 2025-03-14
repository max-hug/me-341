import numpy as np
import matplotlib.pyplot as plt
import csv
import re
from pyansys import __version__

from ansys.workbench.core import launch_workbench

import generation
import constraints

#wb = launch_workbench(show_gui=True, version="251")
#wb.run_script_file("C:\\Users\\maxhu\\Documents\\VS_Code\\me-341\\project\\scripts\\open.wbjn")

data = []

# Read the CSV file
with open("doe_v2.csv", "r") as file:
    reader = csv.reader(file)
    dvs_list = [list(map(str, row)) for row in reader]  # Change `str` to `float` if numbers
dvs_list= np.array(dvs_list, dtype = float)
print("CSV data loaded:")
dvs_list = [row[row != 0].tolist() for row in dvs_list]

max_length= 18

num = 1

print("Loop Started")

for dvs in dvs_list:

    num_slats = len(dvs)-6
    slat_angles = dvs[0:num_slats]
    slat_length = dvs[num_slats]
    frame_x_control = [0, dvs[-5], dvs[-4], 1]
    frame_y_control = [dvs[-3], dvs[-2], dvs[-1], 0]

    frame_control = [frame_x_control, frame_y_control]
    [airfoil_x, airfoil_y], te_slats_used = generation.generate_airfoil(frame_control, slat_length, slat_angles, False)

    export_airfoil = np.array([airfoil_x, airfoil_y]).T
    np.savetxt('airfoil.txt', export_airfoil, delimiter=',', fmt='%.3f', header='', comments='')

    #wb.run_script_file("C:\\Users\\maxhu\\Documents\\VS_Code\\me-341\\project\\scripts\\update.wbjn")

    with open("C:\\Users\\maxhu\\Documents\\VS_Code\\me-341\\project\\scripts\\dp_data.csv", "r", encoding="utf-8-sig") as f:
        report = np.loadtxt(f, delimiter=',', usecols=(1, 2, ), skiprows=7)

    padded_dp =  [*dvs, *([0.0] * (max_length - len(dvs))), *report]
    data.append(padded_dp)
    np.savetxt('cfd_data_v3.csv', data, delimiter=',')
    print("Run {} Complete, data added = {:.3f}, {:.3f}".format(num, data[-1][-2], data[-1][-1]))

    num+=1

#wb.run_script_file("C:\\Users\\maxhu\\Documents\\VS_Code\\me-341\\project\\scripts\\close.wbjn")
#wb.exit()






