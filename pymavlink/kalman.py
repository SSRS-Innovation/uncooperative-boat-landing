import numpy as np




def linearPrediction(x, P, A, Q):
    #LINEARPREDICTION calculates mean and covariance of predicted state
    #   density using a liear Gaussian model.

    x = A*x
    P = A*P*A.transpose()+Q
    return x,P

def linear_update(x, P, y, H, R):
    #   %LINEARPREDICTION calculates mean and covariance of predicted state
    #   density using a linear Gaussian model.

    S = H*P*H.transpose()+R # innovation cov
    K_k = P*H.transpose()*np.linalg.inv(S) # Kalman gain
    vk = y - H*x; # innovation 

    x = x+K_k*vk
    P = P-K_k*S*K_k.transpose()
    return x, P

def kalman_filter(y, x, P, A, Q, H, R):

    # prediction step
    x,P = linearPrediction(x, P, A, Q)

    # update step
    x, P = linear_update(x, P, y, H, R)


    return x, P


