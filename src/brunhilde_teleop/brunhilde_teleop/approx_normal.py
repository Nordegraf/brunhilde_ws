
import numpy as np
# from tf2_geometry_msgs.tf2_geometry_msgs import 

ordering = ['FL', 'FR', 'HL', 'HR']

xy_offsets = {
    'FL': (0.0, 0.0),
    'FR': (0.0, 0.0),
    'HL': (0.0, 0.0),
    'HR': (0.0, 0.0),
}

leg_length = 4

def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

def get_thetas() -> list[tuple[str, float, float]]:\
    '''
    Gets the joint angles from the 3 legs with established ground contact.
    '''
    

def calc_xz_offset(theta_shoulder, theta_knee):
    # TODO:
    # z shifts down by default
    # x sign depends on the used leg
    upper_z = leg_length * np.cos(theta_shoulder)
    upper_x = leg_length * np.sin(theta_shoulder)

    lower_z = leg_length * np.cos(theta_shoulder + theta_knee)
    lower_x = leg_length * np.sin(theta_shoulder + theta_knee)

    return upper_x + lower_x, upper_z + lower_z

def approx_normal(thetas: list[float]):
    points: list[tuple[float, float, float]] = []
    for (leg_id, theta_shoulder, theta_knee) in thetas:
        x_off, z_off = calc_xz_offset(theta_shoulder, theta_knee)
        x_bas, y_bas = xy_offsets(leg_id)
        x = x_bas + x_off
        y = y_bas
        z = z_off
        points.append((x, y, z))
    
    [v_a, v_b, v_c] = points
    base_1 = v_a - v_b
    base_2 = v_c - v_b
    normal = np.cross(base_1, base_2)
    # TODO:
    # the z axis needs to be rotated away 1 alpha from the normal
    return np.subtract((0, 0, 1), normal)


def main():
    thetas = get_thetas()
    approx_normal(thetas)

if __name__ == '__main__':
    main()