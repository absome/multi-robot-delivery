from classes.robot import Robot
from classes.visualize_sim import SimViz
import matplotlib.pyplot as plt
import numpy as np
import cvxopt
import cvxopt.solvers
def get_obstacle_centers(obstacles):
    """
    Calculate the centroid (geometric center) of a polygon given its vertices.
    
    Parameters:
    vertices (np.ndarray): A numpy array of shape (n, 2) representing the polygon's vertices.
    
    Returns:
    tuple: The (x, y) coordinates of the centroid.
    """
    obstacle_center_points = []
    for vertices in obstacles:
        n = len(vertices)
        sum_area = 0.0
        sum_cx = 0.0
        sum_cy = 0.0

        for i in range(n):
                xi, yi = vertices[i]
                xj, yj = vertices[(i + 1) % n]
                contrib = xi * yj - xj * yi
                sum_area += contrib
                sum_cx += (xi + xj) * contrib
                sum_cy += (yi + yj) * contrib

        area = 0.5 * abs(sum_area)
        if area == 0:
                raise ValueError("The polygon has zero area, centroid is undefined.")

        centroid_x = sum_cx / (6 * area)
        centroid_y = sum_cy / (6 * area)
        obstacle_center_points.append((centroid_x, centroid_y))
    return obstacle_center_points

def qp_control(robots, pairs):
    """
    Centralised CBF-QP for 4 planar robots with one circular obstacle.
    Each robot must keep rigid formation and avoid the obstacle.
    """
    N = len(robots)                # should be 4
    assert N == 4, "code below assumes 4 robots"

    # ---------- nominal controls ----------
    u_gtg = np.vstack([r.compute_ugtg(caster=True)[:2] for r in robots])   # (4,2)
    u_gtg_flat = u_gtg.reshape(-1)                                         # (8,)

    Q  = 2*np.eye(2*N)                     # 8×8
    c  = -2*u_gtg_flat                     # 8×1

    # ---------- barrier parameters ----------
    obs_pos   = np.array([0.0, 0.25])
    R_s       = 1.0
    gamma_f   = 0.5
    gamma_o   = 0.2
    eps       = 0.1
    d_ref     = 1.0

    # ---------- how many constraints ----------
    n_edge_constraints = len(pairs)*2
    n_obst_constraints = N
    n_constr = n_edge_constraints + n_obst_constraints

    H = np.zeros((n_constr, 2*N))
    b = np.zeros((n_constr, 1))
    row = 0

    # ---------- formation constraints ----------
    for (i_id, j_id) in pairs:
        # make indices 0-based
        i, j = i_id-1, j_id-1

        s_i = robots[i].state[:2]
        s_j = robots[j].state[:2]
        diff = s_i - s_j
        dij2 = diff @ diff

        # upper-distance barrier
        h1 = -(dij2) + (d_ref + eps)**2
        H[row, 2*i:2*i+2] =  2*diff
        H[row, 2*j:2*j+2] = -2*diff
        b[row, 0]         =  gamma_f * h1
        row += 1

        # lower-distance barrier
        h2 =  dij2 - (d_ref - eps)**2
        H[row, 2*i:2*i+2] = -2*diff
        H[row, 2*j:2*j+2] =  2*diff
        b[row, 0]         =  gamma_f * h2
        row += 1

    # ---------- obstacle constraints ----------
    for k in range(N):
        s_k = robots[k].state[:2]
        diff_o = s_k - obs_pos
        h_obs  = diff_o @ diff_o - R_s**2
        H[row, 2*k:2*k+2] = -2*diff_o
        b[row, 0]         =  gamma_o * h_obs
        row += 1

    # ---------- solve ----------
    Q_mat = cvxopt.matrix(Q, tc='d')
    c_mat = cvxopt.matrix(c, (2*N,1), tc='d')
    H_mat = cvxopt.matrix(H, tc='d')
    b_mat = cvxopt.matrix(b, tc='d')

    cvxopt.solvers.options['show_progress'] = False
    sol = cvxopt.solvers.qp(Q_mat, c_mat, H_mat, b_mat)

    u_opt = np.array(sol['x']).reshape(N,2)     # one row per robot

    return print(u_opt)

        
        
goal_state = np.array([4., 0., 0.])
    

obst1 = np.array( [ [-1., -1.5], [-0.5, -1.5],\
        [-0.5, -1.], [-1., -1.], [-1, -1.5] ]) 
obst2 = np.array( [ [-1., 1.5], [-0.5, 1.5],\
        [-0.5, 1.], [-1., 1.], [-1, 1.5] ])

obstacles = [obst1, obst2]

robot1_state = np.array([-3.75, 0.5, 0.])
robot2_state = np.array([-3.75, -0.25, 0.])
robot3_state = np.array([-3.25, 0.5, 0.])
robot4_state = np.array([-3.25, -0.25, 0.])

robot1 = Robot(robot_id=1, initial_state=robot1_state, goal_state=goal_state, color='r')
robot2 = Robot(robot_id=2, initial_state=robot2_state, goal_state=goal_state, color='b')
robot3 = Robot(robot_id=3, initial_state=robot3_state, goal_state=goal_state, color='g')
robot4 = Robot(robot_id=4, initial_state=robot4_state, goal_state=goal_state, color='y')

robo_pairs = [(1,3), (1,4), (1,2), (2,4), (2,3), (4,3)]

robots = [robot1, robot2, robot3, robot4]
for r in robots:
        r.add_neighbours(robots=robots, pairs=robo_pairs)
# qp_control(robots=robots, pairs=robo_pairs)
# robot1.register_other_robots(robots)
# robot2.register_other_robots(robots)
# robot3.register_other_robots(robots)
# robot4.register_other_robots(robots)

field = ((-4.0, 4.0), (-3.0, 3.0)) # (-x, x), (-y, y)


sim = SimViz(robots=robots, static_obstacles=obstacles, field=field)
sim.init_static_obstacles(obstacles)
sim.run()