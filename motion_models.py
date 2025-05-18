import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        return np.eye(3)

    def control_input_matrix_B(mu, delta_t):
        theta=mu[2]
        B=np.array([
            [np.cos(theta)*delta_t,0],
            [np.sin(theta)*delta_t,0],
            [0,delta_t]
        ])
        return B

    return state_transition_matrix_A, control_input_matrix_B
def velocity_motion_model_2():
    def A():
        A_matrix=np.eye(6)
        A_matrix[0, 3] = dt
        A_matrix[1,4] = dt
        A_matrix[2, 5] = dt
        return A_matrix

    def B(mu, dt):
        return np.zeros((6,2))
        

    return A, B
