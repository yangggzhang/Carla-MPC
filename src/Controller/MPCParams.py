import numpy as np

class MPCParams:
    # State Cost
    Q = np.eye(4)

    # Terminal Cost
    Qf = np.eye(4)

    # Control Cost 1) acceleration 2) steer rate
    R = np.eye(2)

    # State change cost
    Rd = np.eye(2)

    # Horizon
    len_horizon = 10

    # Constrains
    max_steering_angle = 1.0

    a_max = 2

    a_min = -1
    
    a_rate_max = 1
    
    steer_rate_max = 0.5
    
    v_min = -1
    
    v_max = 80

