import numpy as np

class MPCParams:
    # State Cost
    # Q = np.eye(4)
    Q = np.array([[  1.2,  0,  0,  0],
                  [  0,  1.2,  0,  0],
                  [  0,  0,  1.0,  0],
                  [  0,  0,  0,  1.0]])

    # Terminal Cost
    Qf = np.array([[  1.5,  0,  0,  0],
                  [  0,  1.5,  0,  0],
                  [  0,  0,  1.5,  0],
                  [  0,  0,  0,  1.0]])

    # Control Cost 1) acceleration 2) steer rate
    R = np.eye(2)

    dist = 1.5

    # State change cost
    Rd = np.eye(2)

    # Horizon
    len_horizon = 10

    # Constrains
    max_steering_angle = 1.0

    a_max = 5

    a_min = -0.01
    
    a_rate_max = 1
    
    steer_rate_max = 0.5
    
    v_min = -1
    
    v_max = 80

