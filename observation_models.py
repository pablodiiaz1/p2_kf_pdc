import numpy as np

def odometry_observation_model():
    return np.eye(3)

def odometry_observation_model_2():
    # TODO: Return identity matrix (6x6) if all 6 state variables are observed
    return np.eye()
