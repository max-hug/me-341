import numpy as np
import generation
from shapely.geometry import Point, Polygon

### HELPER FUNCTIONS ###

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

### MAIN FUNCTIONS ###

def angle(airfoil_coords):

    max_angle = -10

    for i in range(0, len(airfoil_coords[0])-2):
        points = np.array(list(zip(airfoil_coords[0][i:i+3], airfoil_coords[1][i:i+3])))
        angle = find_angle(points)
        max_angle = max(max_angle,angle)

    return -max_angle

def frame(airfoil_coords, frame_control):

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

    return -sum(not polygon.contains(Point(p)) for p in frame_samples)/20.0 # 20 since 20 samples

def te_slat(slats_used, slat_angles):
    num_slats = len(slat_angles)
    return 1.0*(slats_used-num_slats)/num_slats

def max_reflex(slat_angles):
    return 5*np.pi/4 - sum(np.pi-np.array(slat_angles))

def min_reflex(slat_angles):
    return sum(np.pi-np.array(slat_angles)) - 3*np.pi/4

def connector_length(airfoil_coords, slat_angles, slat_length):
    num_slats = len(slat_angles)
    point1 = np.array(airfoil_coords[0][num_slats], airfoil_coords[1][num_slats])
    point2 = np.array(airfoil_coords[0][num_slats+1], airfoil_coords[1][num_slats+1])
    dist = np.linalg.norm(point2-point1)
    return 7*slat_length - dist

def angle_opt(x):
    airfoil_coords, te_slats_used, frame_control, num_slats, slat_length, slat_angles = generation.opt_generate(x)
    return angle(airfoil_coords)

def frame_opt(x):
    airfoil_coords, te_slats_used, frame_control, num_slats, slat_length, slat_angles = generation.opt_generate(x)
    return frame(airfoil_coords, frame_control)

def te_slat_opt(x):
    airfoil_coords, te_slats_used, frame_control, num_slats, slat_length, slat_angles = generation.opt_generate(x)
    return te_slat(te_slats_used, slat_angles)

def max_reflex_opt(x):
    airfoil_coords, te_slats_used, frame_control, num_slats, slat_length, slat_angles = generation.opt_generate(x)
    return max_reflex(slat_angles)

def min_reflex_opt(x):
    airfoil_coords, te_slats_used, frame_control, num_slats, slat_length, slat_angles = generation.opt_generate(x)
    return min_reflex(slat_angles)

def connector_length_opt(x):
    airfoil_coords, te_slats_used, frame_control, num_slats, slat_length, slat_angles = generation.opt_generate(x)
    return connector_length(airfoil_coords, slat_angles, slat_length)