import numpy as np
import matplotlib.pyplot as plt

### HELPER FUNCTIONS ###

bezier_3 = lambda x, t: x[0]*(1-t)**3 + 3*x[1]*t*(1-t)**2 + 3*x[2]*t**2*(1-t)+x[3]*t**3

def find_t(control_points, start_point, dist, dist_thresh, t_min, t_max):

    if(np.linalg.norm(np.array(start_point)-np.array([bezier_3(control_points[0], 0), bezier_3(control_points[1], 0)])) < dist):
        return -1

    t_avg = (t_max+t_min)/2.0

    curr_point =  [bezier_3(control_points[0], t_avg), bezier_3(control_points[1], t_avg)]

    cand_dist = np.linalg.norm(np.array(start_point)-np.array(curr_point))


    if(abs(dist - cand_dist) < dist_thresh):
        return t_avg
    elif(cand_dist < dist): #move further away, towards t = 0
        return find_t(control_points, start_point, dist, dist_thresh, t_min, t_avg)
    elif(cand_dist > dist): #move closer, towards t = t_max
        return find_t(control_points, start_point, dist, dist_thresh, t_avg, t_max)

def generate_le (start, slat_length, thetas):
    x_coords = [start[0]]
    y_coords = [start[1]]

    alpha = np.pi

    for theta in thetas:
        beta = theta - (np.pi - alpha)
        x_coords.append(x_coords[-1] + slat_length*np.cos(beta))
        y_coords.append(y_coords[-1] + slat_length*np.sin(beta))
        alpha = beta

    return [x_coords, y_coords]

def generate_te(control_points, slat_length, num_slats):
    x_coords = [bezier_3(control_points[0], 1)]
    y_coords = [bezier_3(control_points[1], 1)]

    t_curr = 1
    slat = 0
    while(slat <= num_slats and t_curr >= 0):
        t = find_t(control_points, [x_coords[0], y_coords[0]], slat_length, 0.01, 0, t_curr)
        if(t > 0):
            x_coords.insert(0, bezier_3(control_points[0], t))
            y_coords.insert(0, bezier_3(control_points[1], t))
            slat += 1
        t_curr = t
    if(slat < num_slats): # add half resting one
        apex = [control_points[0][0], control_points[1][0]]
        apex_dist = np.linalg.norm(np.array(apex)-np.array([x_coords[0], y_coords[0]]))
        apex_x_dist = x_coords[0]-apex[0]
        apex_y_dist = y_coords[0]-apex[1]
        scaling_factor = 1.0*slat_length/apex_dist
        x_coords.insert(0, x_coords[0] - scaling_factor*apex_x_dist)
        y_coords.insert(0, y_coords[0] - scaling_factor*apex_y_dist)
        slat += 1

    return x_coords, y_coords, slat

### MAIN FUNCTIONS ###

def add_points(coords, num_points):
    num_points/=2
    airfoil_x = coords[0]
    airfoil_y = coords[1]
    while(len(airfoil_x) < num_points):
        airfoil_x = np.insert(airfoil_x, len(airfoil_x)-1, (airfoil_x[-2]+airfoil_x[-1])/2.0)
        airfoil_y = np.insert(airfoil_y, len(airfoil_y)-1, (airfoil_y[-2]+airfoil_y[-1])/2.0)
    return np.array([airfoil_x, airfoil_y])

def generate_airfoil(frame_control, slat_length, slat_angles, plot):

    num_slats = len(slat_angles)

    [x_le, y_le] = np.array(generate_le([-1, 0], slat_length, slat_angles))
    x_te, y_te, slats_used = generate_te(frame_control, slat_length, num_slats)

    airfoil_x, airfoil_y = [*x_le, *x_te], [*y_le, *y_te]

    if plot:

        t = np.linspace(0.0, 1, 20) # since its touching at the ends, dont start at them
        bezier_3 = lambda x, t: x[0]*(1-t)**3 + 3*x[1]*t*(1-t)**2 + 3*x[2]*t**2*(1-t)+x[3]*t**3
        frame_x = np.array(bezier_3(frame_control[0], t))
        frame_y = np.array(bezier_3(frame_control[1], t))

        fig, axs = plt.subplots(1, 1)

        axs.plot(-1*frame_x, frame_y, linestyle='--', color='blue', label='Frame')
        axs.plot(frame_x, frame_y, linestyle='--', color='blue')
        axs.plot(x_le, y_le, label='LE')
        axs.plot(x_te, y_te, label='TE')
        axs.plot([x_le[-1], x_te[0]], [y_le[-1], y_te[0]], label='Connector')
        axs.plot([-1, 1], [0,0], label='Base')
        axs.set_aspect('equal', 'box')
        axs.set_xlim([-1.25,1.25])
        axs.set_ylim([-0.05,0.7])
        axs.legend()
        plt.show()

    return [[np.array(airfoil_x), np.array(airfoil_y)], slats_used]

def opt_generate(x, plot=False):
    zeros = np.count_nonzero(x == 0)
    num_slats = len(x) - zeros - 6
    num_factors = len(x)
    frame_x_control = [0, x[num_factors-zeros-5], x[num_factors-zeros-4], 1]
    frame_y_control = [x[num_factors-zeros-3], x[num_factors-zeros-2], x[num_factors-zeros-1], 0]
    frame_control = [frame_x_control, frame_y_control]
    slat_length = x[num_slats]

    [airfoil_x, airfoil_y], te_slats_used = generate_airfoil(frame_control, slat_length, x[0:num_slats], plot)
    return [airfoil_x, airfoil_y], te_slats_used, frame_control, num_slats, slat_length, x[0:num_slats]


